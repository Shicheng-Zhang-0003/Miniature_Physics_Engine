#include <gtk/gtk.h>
#include "mpe_engine.h"
camera main_camera_fov;
input_status main_inputs;
static guint physics_timeout_id = 0;

static void on_main_window_destroy(GtkWidget *widget, gpointer user_data) {
    (void) widget;
    (void) user_data;
    if (physics_timeout_id) {
        g_source_remove(physics_timeout_id);
        physics_timeout_id = 0;
    }
    render_cleanup();
    broadphase_cleanup();
    gtk_main_quit();
}
//On Call
static void when_realised(GtkGLArea *gl_area_widget) {
    if (gtk_gl_area_get_error(gl_area_widget) != NULL) {
        return;
    }
    gtk_gl_area_make_current(gl_area_widget);
    //Init OpenGL Status
    glEnable(GL_DEPTH_TEST); //Test Depth Signal
    render_init();
    //Scene Init (On Realize)
    scene_init_default();
} //On render: Screen Make
static gboolean on_rendered(GtkGLArea *gl_area_widget, GdkGLContext *gl_context_data) {
    (void) gl_context_data;
    int screen_scale_factor = gtk_widget_get_scale_factor(GTK_WIDGET(gl_area_widget));
    int widget_width = gtk_widget_get_allocated_width(GTK_WIDGET(gl_area_widget)) * screen_scale_factor;
    int widget_height = gtk_widget_get_allocated_height(GTK_WIDGET(gl_area_widget)) * screen_scale_factor;
    if ((widget_width <= 0) || (widget_height <= 0)) {
        return TRUE;
    }
    render_scene_current(widget_width, widget_height);
    return TRUE;
}
int main_algorithm(int argc, char *argv[]);
int main_algorithm(int argc, char *argv[]) {
    g_setenv("GDK_BACKEND", "x11", TRUE);
    gtk_init(&argc, &argv);
    mpe_config_init(); /* MPE_TASK_29_CONFIG_INIT */
    event_log_init(); /* MPE_TASK_V15R2_EVENT_LOG_INIT */
    /* MPE_TASK_34_CONFIG_LOAD_BEGIN */
    if (mpe_config_load("status/engine.cfg")) {
        printf("[config] loaded status/engine.cfg\n");
    } else {
        printf("[config] defaults active (no saved config)\n");
    }
    /* MPE_TASK_34_CONFIG_LOAD_END */
    printf("MPE %s\n", a3_version_string); /* A3_PATCH_41_FINAL_VALIDATION */
    //Camera Init
    initialize_camera(&main_camera_fov, (vector3){0.0f, 20.0f, 50.0f});
    initialize_input(&main_inputs);
    //Widgeting
    GtkWidget *main_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    g_signal_connect(main_window, "destroy", G_CALLBACK(on_main_window_destroy), NULL);
    GtkWidget *gl_area_widget = gtk_gl_area_new();
    gtk_gl_area_set_has_depth_buffer(GTK_GL_AREA(gl_area_widget), TRUE);
    //Keyboard and Mouse Events
    gtk_widget_add_events(main_window, GDK_KEY_PRESS_MASK | GDK_KEY_RELEASE_MASK | GDK_POINTER_MOTION_MASK |
                                           GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK);
    //Signalling
    g_signal_connect(gl_area_widget, "render", G_CALLBACK(on_rendered), NULL);
    g_signal_connect(gl_area_widget, "realize", G_CALLBACK(when_realised), NULL);
    g_signal_connect(main_window, "key-press-event", G_CALLBACK(on_keypress), &main_inputs);
    g_signal_connect(main_window, "key-release-event", G_CALLBACK(on_key_released), &main_inputs);
    g_signal_connect(main_window, "focus-out-event", G_CALLBACK(on_focus_out), &main_inputs);
    g_signal_connect(gl_area_widget, "focus-out-event", G_CALLBACK(on_focus_out), &main_inputs);
    g_signal_connect(main_window, "motion-notify-event", G_CALLBACK(on_mouse_movements), NULL);
    g_signal_connect(main_window, "button-press-event", G_CALLBACK(on_button_press), &main_inputs);
    g_signal_connect(main_window, "button-release-event", G_CALLBACK(on_button_release), &main_inputs);
    //Add Objects
    GtkWidget *ui_overlay_layout = overlay_initialise(gl_area_widget);
    gtk_container_add(GTK_CONTAINER(main_window), ui_overlay_layout);
    //Focus and Event Catching
    gtk_widget_set_can_focus(main_window, TRUE);
    gtk_widget_grab_focus(main_window);
    //Physics Step Loop (16ms)
    physics_timeout_id = g_timeout_add(16, physics_step_increment, gl_area_widget);
    //Show Window
    gtk_widget_show_all(main_window);
    gtk_widget_grab_focus(main_window);
    frame_timer_init(&main_timer);
    gtk_main();
    /* MPE_TASK_34_CONFIG_SAVE_BEGIN */
    mpe_config_save("status/engine.cfg");
    printf("[config] saved status/engine.cfg\n");
    /* MPE_TASK_34_CONFIG_SAVE_END */
    return 0;
}
int main(int argc, char *argv[]) {
    main_algorithm(argc, argv);
    return 0;
}
