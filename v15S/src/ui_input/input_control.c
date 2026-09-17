/* GTK4: Full port — mirrors GTK3 logic with GtkEventControllerKey/Motion/Gesture signatures. */
#ifdef MPE_GTK4

#include "../mpe_engine.h"
#include "input_control.h"
#include "input_state.h"
#include "camera.h"
#include "mouse_lock.h"
#include <gdk/gdkkeysyms.h>
extern camera main_camera_fov;
extern int selected_object;
void initialize_input(input_status *s) {if(!s)return; s->w_key_pressed=false; s->a_key_pressed=false; s->s_key_pressed=false; s->d_key_pressed=false; s->space_key_pressed=false; s->shift_key_pressed=false; s->escape_key_pressed=false; s->f_key_pressed=false; s->r_key_pressed=false; s->i_key_pressed=false; s->j_key_pressed=false; s->k_key_pressed=false; s->l_key_pressed=false; s->is_menu_open=false; s->menu_1_pressed=false; s->menu_2_pressed=false; s->menu_3_pressed=false; s->menu_4_pressed=false; s->menu_5_pressed=false; s->menu_6_pressed=false; s->spawner_menu_level=0; s->velocity_menu_level=0; s->object_menu_level=0; s->current_spawn_type=0; s->up_arrow_pressed=false; s->down_arrow_pressed=false; s->enter_key_pressed=false; s->e_key_pressed=false; s->stability_test_pressed=false; s->sleep_wake_test_pressed=false; s->editor_torture_pressed=false; s->spawn_stress_pressed=false; s->validation_report_pressed=false; s->debug_terminal_pressed=false; s->long_run_validation_pressed=false; s->config_torture_pressed=false; s->is_mouse_locked=false; s->is_debug_mode_active=false; s->right_mouse_button_clicked=false; s->middle_mouse_button_clicked=false; s->mouse_delta_x=0; s->mouse_delta_y=0; s->suppress_mouse_delta=false; s->marked_joint_object_index=-1; s->enter_spawn_held=false; }
void input_control_attach_controllers(GtkWidget *w, gpointer ud) {(void)w;(void)ud;}
gboolean on_keypress(GtkEventControllerKey *w, guint keyval, guint keycode, GdkModifierType state, gpointer user_data_stored) {
    (void)w; (void)keycode; (void)state;
    input_status *st = (input_status*)user_data_stored;
    if (keyval==GDK_KEY_w||keyval==GDK_KEY_W) st->w_key_pressed=true;
    if (keyval==GDK_KEY_a||keyval==GDK_KEY_A) st->a_key_pressed=true;
    if (keyval==GDK_KEY_s||keyval==GDK_KEY_S) st->s_key_pressed=true;
    if (keyval==GDK_KEY_d||keyval==GDK_KEY_D) st->d_key_pressed=true;
    if (keyval==GDK_KEY_e||keyval==GDK_KEY_E) st->e_key_pressed=true;
    if (keyval==GDK_KEY_f||keyval==GDK_KEY_F) st->f_key_pressed=true;
    if (keyval==GDK_KEY_r||keyval==GDK_KEY_R) st->r_key_pressed=true;
    if (keyval==GDK_KEY_F5) st->stability_test_pressed=true;
    if (keyval==GDK_KEY_F6) st->sleep_wake_test_pressed=true;
    if (keyval==GDK_KEY_F7) st->editor_torture_pressed=true;
    if (keyval==GDK_KEY_F8) st->spawn_stress_pressed=true;
    if (keyval==GDK_KEY_F9) st->validation_report_pressed=true;
    if (keyval==GDK_KEY_F10) st->long_run_validation_pressed=true;
    if (keyval==GDK_KEY_F11) st->config_torture_pressed=true;
    if (keyval==GDK_KEY_i||keyval==GDK_KEY_I) st->i_key_pressed=true;
    if (keyval==GDK_KEY_j||keyval==GDK_KEY_J) st->j_key_pressed=true;
    if (keyval==GDK_KEY_k||keyval==GDK_KEY_K) st->k_key_pressed=true;
    if (keyval==GDK_KEY_l||keyval==GDK_KEY_L) st->l_key_pressed=true;
    if ((keyval==GDK_KEY_9)&&!config_menu_is_open()) {st->spawner_menu_level=0; st->velocity_menu_level=0; st->object_menu_level=0; st->is_menu_open=!st->is_menu_open;}
    if ((keyval==GDK_KEY_8)&&!config_menu_is_open()) {st->is_menu_open=false; st->velocity_menu_level=0; st->object_menu_level=0; if(st->spawner_menu_level>0) st->spawner_menu_level=0; else st->spawner_menu_level=1;}
    if ((keyval==GDK_KEY_7)&&!config_menu_is_open()) {st->is_menu_open=false; st->spawner_menu_level=0; st->object_menu_level=0; if(st->velocity_menu_level>0) st->velocity_menu_level=0; else st->velocity_menu_level=1;}
    if ((keyval==GDK_KEY_6)&&!st->is_menu_open && st->object_menu_level==0) {st->spawner_menu_level=0; st->velocity_menu_level=0; st->object_menu_level=0; if(config_menu_is_open()) config_menu_close(); else config_menu_level_force_open();}
    if ((keyval==GDK_KEY_1)&&!config_menu_is_open() && st->is_debug_mode_active && !st->is_menu_open && st->spawner_menu_level==0 && st->velocity_menu_level==0 && st->object_menu_level==0) st->debug_terminal_pressed=true;
    if (st->is_menu_open) {if(keyval==GDK_KEY_1) st->menu_1_pressed=true; if(keyval==GDK_KEY_2) st->menu_2_pressed=true; if(keyval==GDK_KEY_3) st->menu_3_pressed=true; if(keyval==GDK_KEY_4) st->menu_4_pressed=true; if(keyval==GDK_KEY_5) st->menu_5_pressed=true; if(keyval==GDK_KEY_6) st->menu_6_pressed=true;}
    if (config_menu_is_open()) {if(keyval==GDK_KEY_0) config_menu_key_press(0); if(keyval==GDK_KEY_1) config_menu_key_press(1); if(keyval==GDK_KEY_2) config_menu_key_press(2); if(keyval==GDK_KEY_3) config_menu_key_press(3); if(keyval==GDK_KEY_4) config_menu_key_press(4); if(keyval==GDK_KEY_5) config_menu_key_press(5); if(keyval==GDK_KEY_7) config_menu_key_press(7); if(keyval==GDK_KEY_8) config_menu_key_press(8); if(keyval==GDK_KEY_9) config_menu_key_press(9);}
    if ((st->spawner_menu_level>0)||(st->velocity_menu_level>0)||(st->object_menu_level>0)) {if(keyval==GDK_KEY_Up) st->up_arrow_pressed=true; if(keyval==GDK_KEY_Down) st->down_arrow_pressed=true; if(keyval==GDK_KEY_Return||keyval==GDK_KEY_KP_Enter) st->enter_key_pressed=true;}
    if (st->spawner_menu_level==1) {if(keyval==GDK_KEY_1) st->spawner_menu_level=2; if(keyval==GDK_KEY_2) st->spawner_menu_level=5; if(keyval==GDK_KEY_3) st->spawner_menu_level=8; if(keyval==GDK_KEY_4) st->spawner_menu_level=9;}
    else if (st->spawner_menu_level==2) {if(keyval==GDK_KEY_1) st->spawner_menu_level=3; if(keyval==GDK_KEY_2) st->spawner_menu_level=4;}
    else if (st->spawner_menu_level==5) {if(keyval==GDK_KEY_1) st->spawner_menu_level=6; if(keyval==GDK_KEY_2) st->spawner_menu_level=7;}
    else if (st->spawner_menu_level==9) {if(keyval==GDK_KEY_1) st->spawner_menu_level=10; if(keyval==GDK_KEY_2) st->spawner_menu_level=11; if(keyval==GDK_KEY_3) st->spawner_menu_level=12;}
    if (st->velocity_menu_level==1) {if(keyval==GDK_KEY_1) st->velocity_menu_level=2; if(keyval==GDK_KEY_2) st->velocity_menu_level=10; if(keyval==GDK_KEY_3) st->velocity_menu_level=20;}
    else if (st->velocity_menu_level==2) {if(keyval==GDK_KEY_1) st->velocity_menu_level=3; if(keyval==GDK_KEY_2) st->velocity_menu_level=4;}
    else if (st->velocity_menu_level==20) {if(keyval==GDK_KEY_1) st->velocity_menu_level=21; if(keyval==GDK_KEY_2) st->velocity_menu_level=22; if(keyval==GDK_KEY_3) st->velocity_menu_level=23; if(keyval==GDK_KEY_4) st->velocity_menu_level=24; if(keyval==GDK_KEY_5) st->velocity_menu_level=25;}
    else if (st->velocity_menu_level==10) {if(keyval==GDK_KEY_1) st->velocity_menu_level=11; if(keyval==GDK_KEY_2) st->velocity_menu_level=12;}
    if (st->object_menu_level==1) {if(keyval==GDK_KEY_1) st->object_menu_level=2; if(keyval==GDK_KEY_2) st->object_menu_level=3; if(keyval==GDK_KEY_3) st->object_menu_level=4; if(keyval==GDK_KEY_4) st->object_menu_level=5; if(keyval==GDK_KEY_5) st->object_menu_level=6; if(keyval==GDK_KEY_6){if(st->marked_joint_object_index!=-1&&st->marked_joint_object_index!=selected_object) st->object_menu_level=7; else st->object_menu_level=8;} if(keyval==GDK_KEY_7&&!config_menu_is_open()){if(st->marked_joint_object_index!=-1&&st->marked_joint_object_index!=selected_object) st->object_menu_level=8;}}
    else if (st->object_menu_level==8) {if(keyval==GDK_KEY_1) st->object_menu_level=81; if(keyval==GDK_KEY_2) st->object_menu_level=82; if(keyval==GDK_KEY_3) st->object_menu_level=83; if(keyval==GDK_KEY_4) st->object_menu_level=84; if(keyval==GDK_KEY_5) st->object_menu_level=85; if(keyval==GDK_KEY_6) st->object_menu_level=86; if(keyval==GDK_KEY_7&&!config_menu_is_open()) st->object_menu_level=87; if(keyval==GDK_KEY_8&&!config_menu_is_open()) st->object_menu_level=88;}
    if (((keyval==GDK_KEY_Return||keyval==GDK_KEY_KP_Enter)&&!st->is_menu_open&&st->spawner_menu_level==0&&st->velocity_menu_level==0&&st->object_menu_level==0)) st->enter_spawn_held=true;
    if (keyval==GDK_KEY_space||keyval==GDK_KEY_KP_Space) st->space_key_pressed=true;
    if (keyval==GDK_KEY_Shift_L||keyval==GDK_KEY_Shift_R) st->shift_key_pressed=true;
    if (keyval==GDK_KEY_Escape) st->escape_key_pressed=true;
    if ((keyval==GDK_KEY_0||keyval==GDK_KEY_KP_0)&&!config_menu_is_open()) st->is_debug_mode_active=!st->is_debug_mode_active;
    return FALSE;
}
gboolean on_key_released(GtkEventControllerKey *w, guint keyval, guint keycode, GdkModifierType state, gpointer user_data_stored) {
    (void)w; (void)keycode; (void)state;
    input_status *st=(input_status*)user_data_stored;
    if(keyval==GDK_KEY_w||keyval==GDK_KEY_W) st->w_key_pressed=false;
    if(keyval==GDK_KEY_a||keyval==GDK_KEY_A) st->a_key_pressed=false;
    if(keyval==GDK_KEY_s||keyval==GDK_KEY_S) st->s_key_pressed=false;
    if(keyval==GDK_KEY_d||keyval==GDK_KEY_D) st->d_key_pressed=false;
    if(keyval==GDK_KEY_r||keyval==GDK_KEY_R) st->r_key_pressed=false;
    if(keyval==GDK_KEY_i||keyval==GDK_KEY_I) st->i_key_pressed=false;
    if(keyval==GDK_KEY_j||keyval==GDK_KEY_J) st->j_key_pressed=false;
    if(keyval==GDK_KEY_k||keyval==GDK_KEY_K) st->k_key_pressed=false;
    if(keyval==GDK_KEY_l||keyval==GDK_KEY_L) st->l_key_pressed=false;
    if(keyval==GDK_KEY_space||keyval==GDK_KEY_KP_Space) st->space_key_pressed=false;
    if(keyval==GDK_KEY_Shift_L||keyval==GDK_KEY_Shift_R) st->shift_key_pressed=false;
    if(keyval==GDK_KEY_Return||keyval==GDK_KEY_KP_Enter) st->enter_spawn_held=false;
    return FALSE;
}
gboolean on_mouse_movements(GtkEventControllerMotion *ctrl, double x, double y, gpointer ud) {
    (void)ctrl;
    input_status *st=(input_status*)ud;
    if(!st) st=&main_inputs;
    if(!st->is_mouse_locked) return FALSE;
    // Use toplevel size for clamping, not widget (controller has no size)
    // For now use fixed 800x600 heuristic or get from widget via gtk_event_controller_get_widget
    GtkWidget *w = gtk_event_controller_get_widget(GTK_EVENT_CONTROLLER(ctrl));
    int ww = w ? gtk_widget_get_width(w) : 800;
    int wh = w ? gtk_widget_get_height(w) : 600;
    if(ww<=0||wh<=0) {ww=800; wh=600;}
    static double last_x=-1,last_y=-1;
    if(last_x<0) {last_x=x; last_y=y; return FALSE;}
    if(st->suppress_mouse_delta) {last_x=x; last_y=y; return FALSE;}
    double dx = x - last_x;
    double dy = y - last_y;
    if (dx > ww/2) dx = ww/2;
    if (dx < -ww/2) dx = -ww/2;
    if (dy > wh/2) dy = wh/2;
    if (dy < -wh/2) dy = -wh/2;
    if(dx!=0||dy!=0) {
        st->mouse_delta_x = (float)dx;
        st->mouse_delta_y = -(float)dy;
        last_x = x; last_y = y;
    }
    return FALSE;
}
gboolean on_button_press(GtkGestureClick *gest, int n_press, double x, double y, gpointer user_data_stored) {
    (void)x; (void)y; (void)n_press;
    input_status *st=(input_status*)user_data_stored;
    if(!st) st=&main_inputs;
    guint btn = gtk_gesture_single_get_current_button(GTK_GESTURE_SINGLE(gest));
    if(btn==2) st->middle_mouse_button_clicked=true;
    if(btn==3) st->right_mouse_button_clicked=true;
    if(btn==1 && !st->is_mouse_locked) {
        st->mouse_delta_x=0; st->mouse_delta_y=0;
        st->is_mouse_locked=true;
        mouse_lock_enable(gtk_event_controller_get_widget(GTK_EVENT_CONTROLLER(gest)));
    } else if(!st->is_mouse_locked) {
        st->is_mouse_locked=true;
        mouse_lock_enable(gtk_event_controller_get_widget(GTK_EVENT_CONTROLLER(gest)));
    }
    return FALSE;
}
gboolean on_button_release(GtkGestureClick *gest, int n_press, double x, double y, gpointer user_data_stored) {
    input_status *st=(input_status*)user_data_stored;
    if(!st) st=&main_inputs;
    guint btn = gtk_gesture_single_get_current_button(GTK_GESTURE_SINGLE(gest));
    (void)n_press; (void)x; (void)y;
    if(btn==2) st->middle_mouse_button_clicked=false;
    if(btn==3) st->right_mouse_button_clicked=false;
    return FALSE;
}
gboolean on_focus_out(GtkEventControllerFocus *ctrl, gpointer user_data_stored) {
    (void)ctrl;
    input_status *st=(input_status*)user_data_stored;
    if(!st) st=&main_inputs;
    st->w_key_pressed=false; st->a_key_pressed=false; st->s_key_pressed=false; st->d_key_pressed=false; st->space_key_pressed=false; st->shift_key_pressed=false; st->escape_key_pressed=false; st->f_key_pressed=false;
    st->i_key_pressed=false; st->j_key_pressed=false; st->k_key_pressed=false; st->l_key_pressed=false; st->r_key_pressed=false;
    st->up_arrow_pressed=false; st->down_arrow_pressed=false; st->enter_key_pressed=false; st->e_key_pressed=false;
    st->stability_test_pressed=false; st->sleep_wake_test_pressed=false; st->editor_torture_pressed=false; st->spawn_stress_pressed=false; st->validation_report_pressed=false;
    st->long_run_validation_pressed=false; st->config_torture_pressed=false; st->debug_terminal_pressed=false; st->enter_spawn_held=false;
    st->mouse_delta_x=0; st->mouse_delta_y=0; st->right_mouse_button_clicked=false; st->middle_mouse_button_clicked=false; st->suppress_mouse_delta=false;
    if(st->is_mouse_locked) {
        mouse_lock_disable(gtk_event_controller_get_widget(GTK_EVENT_CONTROLLER(ctrl)));
        st->is_mouse_locked=false;
    }
    return FALSE;
}

#else
#include "../mpe_engine.h"
#include "input_control.h"
#include "camera.h"
#include "mouse_lock.h"
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <stdbool.h>
extern camera main_camera_fov;
extern input_status main_inputs;
extern int selected_object;
void initialize_input (input_status *input_state) {
    //Keyboard
    input_state -> w_key_pressed = false;
    input_state -> a_key_pressed = false;
    input_state -> s_key_pressed = false;
    input_state -> d_key_pressed = false;
    input_state -> space_key_pressed = false;
    input_state -> shift_key_pressed = false;
    input_state -> escape_key_pressed = false;
    input_state -> f_key_pressed = false;
    /* q/m/delete/t keybinds REMOVED (dead or deleted; see header). */
    input_state -> i_key_pressed = false;
    input_state -> j_key_pressed = false;
    input_state -> k_key_pressed = false;
    input_state -> l_key_pressed = false;
    /* MPE_TASK_21_KEYBOARD_ONLY_INIT_BEGIN (r only) */
input_state -> r_key_pressed = false;
/* MPE_TASK_21_KEYBOARD_ONLY_INIT_END */
//Menu
    input_state -> is_menu_open = false;
    input_state -> menu_1_pressed = false;
    input_state -> menu_2_pressed = false;
    input_state -> menu_3_pressed = false;
    input_state -> menu_4_pressed = false;
    input_state -> menu_5_pressed = false;
    input_state -> menu_6_pressed = false; /* MPE_TASK_35_FOCUS */
    //Spawn
    input_state -> spawner_menu_level = 0;
    input_state -> velocity_menu_level = 0;
    input_state -> object_menu_level = 0;
    input_state -> current_spawn_type = 0; // 0: Sphere, 1: Cube
    input_state -> up_arrow_pressed = false;
    input_state -> down_arrow_pressed = false;
    input_state -> enter_key_pressed = false;
    input_state -> e_key_pressed = false;
input_state -> stability_test_pressed = false;
input_state -> sleep_wake_test_pressed = false;
input_state -> editor_torture_pressed = false;
input_state -> spawn_stress_pressed = false;
input_state -> validation_report_pressed = false;
    /* MPE_TASK_13_LONG_RUN_INIT_BEGIN */
input_state -> long_run_validation_pressed = false;
/* MPE_TASK_13_LONG_RUN_INIT_END */
/* MPE_TASK_39_CONFIG_TORTURE_INIT_BEGIN */
input_state -> config_torture_pressed = false;
/* MPE_TASK_39_CONFIG_TORTURE_INIT_END */
/* MPE_TASK_18_TERMINAL_INPUT_INIT_BEGIN */
input_state -> debug_terminal_pressed = false;
/* MPE_TASK_18_TERMINAL_INPUT_INIT_END */
/* MPE_TASK_22_ENTER_SPAWN_INIT_BEGIN */
input_state -> enter_spawn_held = false;
/* MPE_TASK_22_ENTER_SPAWN_INIT_END */
//Mouse
    input_state -> is_mouse_locked = false;
    input_state -> is_debug_mode_active = false;
    input_state -> right_mouse_button_clicked = false;
    input_state -> middle_mouse_button_clicked = false;
    input_state -> mouse_delta_x = 0.0f;
    input_state -> mouse_delta_y = 0.0f;
    input_state -> suppress_mouse_delta = false;
    input_state -> marked_joint_object_index = -1;
} gboolean on_keypress (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    (void) widget;
    input_status *input_state = (input_status *) user_data_stored;
    if (event -> keyval == GDK_KEY_w) {input_state -> w_key_pressed = true;}
    if (event -> keyval == GDK_KEY_a) {input_state -> a_key_pressed = true;}
    if (event -> keyval == GDK_KEY_s) {input_state -> s_key_pressed = true;}
    if (event -> keyval == GDK_KEY_d) {input_state -> d_key_pressed = true;}
    if (event -> keyval == GDK_KEY_e) {input_state -> e_key_pressed = true;}
    if (event -> keyval == GDK_KEY_f) {input_state -> f_key_pressed = true;}
/* q/m/delete/t/left/right REMOVED (dead or deleted keybinds; see header). */
/* MPE_TASK_21_KEYBOARD_ONLY_KEYPRESS_BEGIN (r only) */
if ((event -> keyval == GDK_KEY_r) || (event -> keyval == GDK_KEY_R)) {input_state -> r_key_pressed = true;}
/* MPE_TASK_21_KEYBOARD_ONLY_KEYPRESS_END */
if (event -> keyval == GDK_KEY_F5) {input_state -> stability_test_pressed = true;}
if (event -> keyval == GDK_KEY_F6) {input_state -> sleep_wake_test_pressed = true;}
if (event -> keyval == GDK_KEY_F7) {input_state -> editor_torture_pressed = true;}
if (event -> keyval == GDK_KEY_F8) {input_state -> spawn_stress_pressed = true;}
if (event -> keyval == GDK_KEY_F9) {input_state -> validation_report_pressed = true;}
/* MPE_TASK_13_LONG_RUN_KEY_BEGIN */
if (event -> keyval == GDK_KEY_F10) {input_state -> long_run_validation_pressed = true;}
/* MPE_TASK_13_LONG_RUN_KEY_END */
/* MPE_TASK_39_CONFIG_TORTURE_KEY_BEGIN */
if (event -> keyval == GDK_KEY_F11) {input_state -> config_torture_pressed = true;}
/* MPE_TASK_39_CONFIG_TORTURE_KEY_END */
    if (event -> keyval == GDK_KEY_i) {input_state -> i_key_pressed = true;}
    if (event -> keyval == GDK_KEY_j) {input_state -> j_key_pressed = true;}
    if (event -> keyval == GDK_KEY_k) {input_state -> k_key_pressed = true;}
    if (event -> keyval == GDK_KEY_l) {input_state -> l_key_pressed = true;}
    if ((event -> keyval == GDK_KEY_9) && (!config_menu_is_open ())) {input_state -> spawner_menu_level = 0; input_state -> velocity_menu_level = 0; input_state -> object_menu_level = 0; input_state -> is_menu_open = !(input_state -> is_menu_open);}
    if ((event -> keyval == GDK_KEY_8) && (!config_menu_is_open ())) {input_state -> is_menu_open = false; input_state -> velocity_menu_level = 0; input_state -> object_menu_level = 0; if (input_state -> spawner_menu_level > 0) {input_state -> spawner_menu_level = 0;} else {input_state -> spawner_menu_level = 1;}}
    if ((event -> keyval == GDK_KEY_7) && (!config_menu_is_open ())) {input_state -> is_menu_open = false; input_state -> spawner_menu_level = 0; input_state -> object_menu_level = 0; if (input_state -> velocity_menu_level > 0) {input_state -> velocity_menu_level = 0;} else {input_state -> velocity_menu_level = 1;}}
/* MPE_TASK_35_CONFIG_MENU_KEY_BEGIN */
if ((event -> keyval == GDK_KEY_6) && (!input_state -> is_menu_open) && (input_state -> object_menu_level == 0)) {
input_state -> spawner_menu_level = 0;
input_state -> velocity_menu_level = 0;
input_state -> object_menu_level = 0;
if (config_menu_is_open ()) {config_menu_close ();}
else {config_menu_level_force_open ();}
}
/* MPE_TASK_35_CONFIG_MENU_KEY_END */
    /* MPE_TASK_18_TERMINAL_KEY_BEGIN */
if ((event -> keyval == GDK_KEY_1) && (!config_menu_is_open ()) &&
(input_state -> is_debug_mode_active) &&
(!input_state -> is_menu_open) &&
(input_state -> spawner_menu_level == 0) &&
(input_state -> velocity_menu_level == 0) &&
(input_state -> object_menu_level == 0)) {
input_state -> debug_terminal_pressed = true;
}
/* MPE_TASK_18_TERMINAL_KEY_END */
if (input_state -> is_menu_open) {
        if (event -> keyval == GDK_KEY_1) {input_state -> menu_1_pressed = true;}
        if (event -> keyval == GDK_KEY_2) {input_state -> menu_2_pressed = true;}
        if (event -> keyval == GDK_KEY_3) {input_state -> menu_3_pressed = true;}
        if (event -> keyval == GDK_KEY_4) {input_state -> menu_4_pressed = true;}
        if (event -> keyval == GDK_KEY_5) {input_state -> menu_5_pressed = true;}
        if (event -> keyval == GDK_KEY_6) {input_state -> menu_6_pressed = true;} /* MPE_TASK_35 */
    } // Menu Navigation
    /* MPE_TASK_35_CONFIG_MENU_NAV_BEGIN */
    if (config_menu_is_open ()) {
        if (event -> keyval == GDK_KEY_0) {config_menu_key_press (0);}
        if (event -> keyval == GDK_KEY_1) {config_menu_key_press (1);}
        if (event -> keyval == GDK_KEY_2) {config_menu_key_press (2);}
        if (event -> keyval == GDK_KEY_3) {config_menu_key_press (3);}
        if (event -> keyval == GDK_KEY_4) {config_menu_key_press (4);}
        if (event -> keyval == GDK_KEY_5) {config_menu_key_press (5);}
        if (event -> keyval == GDK_KEY_7) {config_menu_key_press (7);}
        if (event -> keyval == GDK_KEY_8) {config_menu_key_press (8);}
        if (event -> keyval == GDK_KEY_9) {config_menu_key_press (9);}
    }
    /* MPE_TASK_35_CONFIG_MENU_NAV_END */
    if ((input_state -> spawner_menu_level > 0) || (input_state -> velocity_menu_level > 0) || (input_state -> object_menu_level > 0)) {
        if (event -> keyval == GDK_KEY_Up) {input_state -> up_arrow_pressed = true;}
        if (event -> keyval == GDK_KEY_Down) {input_state -> down_arrow_pressed = true;}
        /* Left/Right REMOVED (pre-dialog change-rate relics). */
        if ((event -> keyval == GDK_KEY_Return) || (event -> keyval == GDK_KEY_KP_Enter)) {input_state -> enter_key_pressed = true;}
    } // Spawner Menu Logic
    if (input_state -> spawner_menu_level == 1) {
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 2;}
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 5;}
        if (event -> keyval == GDK_KEY_3) {input_state -> spawner_menu_level = 8;}
        if (event -> keyval == GDK_KEY_4) {input_state -> spawner_menu_level = 9;}
    } else if (input_state -> spawner_menu_level == 2) {
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 3;}
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 4;}
    } else if (input_state -> spawner_menu_level == 5) {
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 6;}
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 7;}
    } else if (input_state -> spawner_menu_level == 9) {
        if (event -> keyval == GDK_KEY_1) {input_state -> spawner_menu_level = 10;}
        if (event -> keyval == GDK_KEY_2) {input_state -> spawner_menu_level = 11;}
        if (event -> keyval == GDK_KEY_3) {input_state -> spawner_menu_level = 12;}
    } // Velocity Menu Logic
    if (input_state -> velocity_menu_level == 1) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 2;}
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 10;}
        if (event -> keyval == GDK_KEY_3) {input_state -> velocity_menu_level = 20;}
    } else if (input_state -> velocity_menu_level == 2) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 3;}
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 4;}
    } else if (input_state -> velocity_menu_level == 20) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 21;}
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 22;}
        if (event -> keyval == GDK_KEY_3) {input_state -> velocity_menu_level = 23;}
        /* World-physics shortcuts imported from menu 6 (game-mode-safe,
         * non-debug-only params only; debug-only physics stays in 6). */
        if (event -> keyval == GDK_KEY_4) {input_state -> velocity_menu_level = 24;}
        if (event -> keyval == GDK_KEY_5) {input_state -> velocity_menu_level = 25;}
    } else if (input_state -> velocity_menu_level == 10) {
        if (event -> keyval == GDK_KEY_1) {input_state -> velocity_menu_level = 11;}
        if (event -> keyval == GDK_KEY_2) {input_state -> velocity_menu_level = 12;}
    } // Selected Object Menu Logic
    if (input_state -> object_menu_level == 1) {
        if (event -> keyval == GDK_KEY_1) {input_state -> object_menu_level = 2;}
        if (event -> keyval == GDK_KEY_2) {input_state -> object_menu_level = 3;}
        if (event -> keyval == GDK_KEY_3) {input_state -> object_menu_level = 4;}
        if (event -> keyval == GDK_KEY_4) {input_state -> object_menu_level = 5;}
        if (event -> keyval == GDK_KEY_5) {input_state -> object_menu_level = 6;}
        if (event -> keyval == GDK_KEY_6) {
            if (input_state -> marked_joint_object_index != -1 && input_state -> marked_joint_object_index != selected_object) {
                input_state -> object_menu_level = 7;
            } else {
                input_state -> object_menu_level = 8;
            }
        }
        if ((event -> keyval == GDK_KEY_7) && (!config_menu_is_open ())) {
            if (input_state -> marked_joint_object_index != -1 && input_state -> marked_joint_object_index != selected_object) {
                input_state -> object_menu_level = 8;
            }
        }
    } else if (input_state -> object_menu_level == 8) {
        if (event -> keyval == GDK_KEY_1) {input_state -> object_menu_level = 81;}
        if (event -> keyval == GDK_KEY_2) {input_state -> object_menu_level = 82;}
        if (event -> keyval == GDK_KEY_3) {input_state -> object_menu_level = 83;}
        if (event -> keyval == GDK_KEY_4) {input_state -> object_menu_level = 84;}
        if (event -> keyval == GDK_KEY_5) {input_state -> object_menu_level = 85;}
        if (event -> keyval == GDK_KEY_6) {input_state -> object_menu_level = 86;}
        if ((event -> keyval == GDK_KEY_7) && (!config_menu_is_open ())) {input_state -> object_menu_level = 87;}
        if ((event -> keyval == GDK_KEY_8) && (!config_menu_is_open ())) {input_state -> object_menu_level = 88;}
    } /* MPE_TASK_22_ENTER_SPAWN_KEYPRESS_BEGIN */
if (((event -> keyval == GDK_KEY_Return) || (event -> keyval == GDK_KEY_KP_Enter)) &&
(!input_state -> is_menu_open) &&
(input_state -> spawner_menu_level == 0) &&
(input_state -> velocity_menu_level == 0) &&
(input_state -> object_menu_level == 0)) {
input_state -> enter_spawn_held = true;
}
/* MPE_TASK_22_ENTER_SPAWN_KEYPRESS_END */
if (event -> keyval == GDK_KEY_space) {input_state -> space_key_pressed = true;}
    if (event -> keyval == GDK_KEY_Shift_L) {input_state -> shift_key_pressed = true;}
    if (event -> keyval == GDK_KEY_Escape) {input_state -> escape_key_pressed = true;}
    if ((event -> keyval == GDK_KEY_0) && (!config_menu_is_open ())) {input_state -> is_debug_mode_active = !input_state -> is_debug_mode_active;}
    return FALSE;
} gboolean on_key_released (GtkWidget *widget, GdkEventKey *event, gpointer user_data_stored) {
    (void) widget;
    input_status *input_state = (input_status *) user_data_stored;
    if (event -> keyval == GDK_KEY_w) {input_state -> w_key_pressed = false;}
    if (event -> keyval == GDK_KEY_a) {input_state -> a_key_pressed = false;}
    if (event -> keyval == GDK_KEY_s) {input_state -> s_key_pressed = false;}
    if (event -> keyval == GDK_KEY_d) {input_state -> d_key_pressed = false;}
/* MPE_TASK_21_KEYBOARD_ONLY_KEYRELEASE_BEGIN (r only) */
if ((event -> keyval == GDK_KEY_r) || (event -> keyval == GDK_KEY_R)) {input_state -> r_key_pressed = false;}
/* MPE_TASK_21_KEYBOARD_ONLY_KEYRELEASE_END */
    if (event -> keyval == GDK_KEY_i) {input_state -> i_key_pressed = false;}
    if (event -> keyval == GDK_KEY_j) {input_state -> j_key_pressed = false;}
    if (event -> keyval == GDK_KEY_k) {input_state -> k_key_pressed = false;}
    if (event -> keyval == GDK_KEY_l) {input_state -> l_key_pressed = false;}
/* q/m/delete/t REMOVED (dead or deleted keybinds). */
    if (event -> keyval == GDK_KEY_space) {input_state -> space_key_pressed = false;} /* MFS_154 */
    if (event -> keyval == GDK_KEY_Shift_L) {input_state -> shift_key_pressed = false;} /* MFS_154 */
/* MPE_TASK_22_ENTER_SPAWN_KEYRELEASE_BEGIN */
if ((event -> keyval == GDK_KEY_Return) || (event -> keyval == GDK_KEY_KP_Enter)) {
input_state -> enter_spawn_held = false;
}
/* MPE_TASK_22_ENTER_SPAWN_KEYRELEASE_END */
    return FALSE;
} gboolean on_mouse_movements (GtkWidget *widget, GdkEventMotion *event, gpointer user_data_stored) {
    (void) user_data_stored;
    input_status *input_state = &main_inputs;
    if (input_state -> is_mouse_locked) {
        static int last_warp_x = -1;
        static int last_warp_y = -1;
        int current_mouse_x = (int) event -> x_root;
        int current_mouse_y = (int) event -> y_root;
        if ((current_mouse_x == last_warp_x) && (current_mouse_y == last_warp_y)) {return FALSE;}
        if (input_state -> suppress_mouse_delta) {
            last_warp_x = current_mouse_x;
            last_warp_y = current_mouse_y;
            return FALSE;
        } int widget_width  = gtk_widget_get_allocated_width  (widget);
        int widget_height = gtk_widget_get_allocated_height (widget);
        int center_x = widget_width / 2;
        int center_y = widget_height / 2;
        int screen_origin_x, screen_origin_y;
        gdk_window_get_origin (gtk_widget_get_window (widget), &screen_origin_x, &screen_origin_y);
        int screen_center_x = screen_origin_x + center_x;
        int screen_center_y = screen_origin_y + center_y;
        int delta_x = current_mouse_x - screen_center_x;
        int delta_y = current_mouse_y - screen_center_y;
        int half_w = widget_width / 2;
        int half_h = widget_height / 2;
        if (delta_x >  half_w) {delta_x =  half_w;}
        if (delta_x < -half_w) {delta_x = -half_w;}
        if (delta_y >  half_h) {delta_y =  half_h;}
        if (delta_y < -half_h) {delta_y = -half_h;}
        if ((delta_x != 0) || (delta_y != 0)) {
            input_state -> mouse_delta_x = (float) delta_x;
            input_state -> mouse_delta_y = -(float) delta_y;
            last_warp_x = screen_center_x;
            last_warp_y = screen_center_y;
            mouse_lock_reset_centre (widget);
        }
    } return FALSE;
} gboolean on_button_press (GtkWidget *widget, GdkEventButton *event, gpointer user_data_stored) {
    input_status *input_state = (input_status *) user_data_stored;
    /* Left click only locks the mouse (no selection action attached). */
    if (event -> button == 2) {input_state -> middle_mouse_button_clicked = true;}
    if (event -> button == 3) {input_state -> right_mouse_button_clicked = true;}
    if (!(input_state -> is_mouse_locked)) {
        input_state -> mouse_delta_x = 0.0f;
        input_state -> mouse_delta_y = 0.0f;
        mouse_lock_enable (gtk_widget_get_toplevel (widget));
        input_state -> is_mouse_locked = true;
    } return FALSE;
} gboolean on_button_release (GtkWidget *widget, GdkEventButton *event, gpointer user_data_stored) {
    (void) widget;
    input_status *input_state = (input_status *) user_data_stored;
    if (event -> button == 2) {input_state -> middle_mouse_button_clicked = false;}
    if (event -> button == 3) {input_state -> right_mouse_button_clicked = false;}
    return FALSE;
} gboolean on_focus_out (GtkWidget *widget, GdkEventFocus *event, gpointer user_data_stored) {
    (void) widget;
    (void) event;
    input_status *input_state = (input_status *) user_data_stored;
    input_state -> w_key_pressed = false;
    input_state -> a_key_pressed = false;
    input_state -> s_key_pressed = false;
    input_state -> d_key_pressed = false;
    input_state -> space_key_pressed = false;
    input_state -> shift_key_pressed = false;
    input_state -> escape_key_pressed = false;
    input_state -> f_key_pressed = false;
/* q/m/delete/t + g/h/c/v/b/n REMOVED (dead or deleted keybinds). */
    input_state -> i_key_pressed = false;
    input_state -> j_key_pressed = false;
    input_state -> k_key_pressed = false;
    input_state -> l_key_pressed = false;
    /* MPE_TASK_21_KEYBOARD_ONLY_FOCUS_BEGIN (r only) */
input_state -> r_key_pressed = false;
/* MPE_TASK_21_KEYBOARD_ONLY_FOCUS_END */
input_state -> up_arrow_pressed = false;
    input_state -> down_arrow_pressed = false;
    input_state -> enter_key_pressed = false;
    input_state -> e_key_pressed = false;
input_state -> stability_test_pressed = false;
input_state -> sleep_wake_test_pressed = false;
input_state -> editor_torture_pressed = false;
input_state -> spawn_stress_pressed = false;
input_state -> validation_report_pressed = false;
    /* MPE_TASK_13_LONG_RUN_FOCUS_BEGIN */
input_state -> long_run_validation_pressed = false;
/* MPE_TASK_13_LONG_RUN_FOCUS_END */
/* MPE_TASK_39_CONFIG_TORTURE_FOCUS_BEGIN */
input_state -> config_torture_pressed = false;
/* MPE_TASK_39_CONFIG_TORTURE_FOCUS_END */
/* MPE_TASK_18_TERMINAL_FOCUS_BEGIN */
input_state -> debug_terminal_pressed = false;
/* MPE_TASK_18_TERMINAL_FOCUS_END */
/* MPE_TASK_22_ENTER_SPAWN_FOCUS_BEGIN */
input_state -> enter_spawn_held = false;
/* MPE_TASK_22_ENTER_SPAWN_FOCUS_END */
/* A3_PATCH_02_FOCUS_LOSS */
    input_state -> mouse_delta_x = 0.0f;
    input_state -> mouse_delta_y = 0.0f;
    input_state -> right_mouse_button_clicked = false;
    input_state -> middle_mouse_button_clicked = false;
    input_state -> suppress_mouse_delta = false;

    if (input_state -> is_mouse_locked) {
        GtkWidget *a3_toplevel_widget = gtk_widget_get_toplevel (widget);
        mouse_lock_disable (a3_toplevel_widget);
        input_state -> is_mouse_locked = false;
    }

    return FALSE;
}

#endif /* MPE_GTK4 */
