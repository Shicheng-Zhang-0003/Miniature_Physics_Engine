#include <gtk/gtk.h>
#include "mpe_engine.h"
camera main_camera_fov;
input_status main_inputs;
static guint physics_timeout_id = 0;

#ifdef MPE_GTK4
/* ====================== GTK4 PATH ====================== */
/* Forward: GTK4 needs GtkApplication; keep physics/error handling identical. */
static GtkApplication *mpe_app = NULL;

static void on_main_window_destroy_gtk4(GtkWidget *widget, gpointer user_data) {
    (void) widget;
    (void) user_data;
    if (physics_timeout_id) {
        g_source_remove(physics_timeout_id);
        physics_timeout_id = 0;
    }
    render_cleanup();
    physics_world_cleanup(physics_world_get_primary());
    if (mpe_app) {
        g_application_quit(G_APPLICATION(mpe_app));
    }
}
static void when_realised(GtkGLArea *gl_area_widget) {
    printf("[GTK4] when_realised called\n");
    if (gtk_gl_area_get_error(gl_area_widget) != NULL) {
        printf("[GTK4] gl_area error: %s\n", gtk_gl_area_get_error(gl_area_widget)->message);
        return;
    }
    gtk_gl_area_make_current(gl_area_widget);
    printf("[GTK4] glMakeCurrent ok, init GL\n");
    glEnable(GL_DEPTH_TEST);
    render_init();
    printf("[GTK4] render_init done\n");
    scene_init_default();
    printf("[GTK4] scene_init done, bodies=%d\n", physics_world_get_primary()->body_count);
}
static gboolean on_rendered(GtkGLArea *gl_area_widget, GdkGLContext *gl_context_data) {
    (void) gl_context_data;
    /* GTK4: get_width/height + scale factor. No allocated_* deprecation. */
    int scale = gtk_widget_get_scale_factor(GTK_WIDGET(gl_area_widget));
    int w = gtk_widget_get_width(GTK_WIDGET(gl_area_widget)) * scale;
    int h = gtk_widget_get_height(GTK_WIDGET(gl_area_widget)) * scale;
    if ((w <= 0) || (h <= 0)) {
        return TRUE;
    }
    //printf("[GTK4] on_rendered %dx%d\n", w, h);
    render_scene_current(w, h);
    return TRUE;
}

/* GTK4: use input_control.c full handlers directly. */

static void app_activate(GApplication *app, gpointer user_data) {
    (void) user_data;
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
    printf("[GTK4] app_activate\n"); fflush(stdout);
    mpe_config_init();
    event_log_init();
    if (mpe_config_load("status/engine.cfg")) {
        printf("[config] loaded status/engine.cfg\n"); fflush(stdout);
    } else {
        printf("[config] defaults active (no saved config)\n"); fflush(stdout);
    }
    printf("MPE %s (GTK4)\n", a3_version_string); fflush(stdout);
    physics_world_init(physics_world_get_primary());
    initialize_camera(&main_camera_fov, (vector3){0.0f, 20.0f, 50.0f});
    initialize_input(&main_inputs);

    GtkWidget *main_window = gtk_application_window_new(GTK_APPLICATION(app));
    gtk_window_set_title(GTK_WINDOW(main_window), "MPE v15R3 — GTK4");
    gtk_window_set_default_size(GTK_WINDOW(main_window), 1280, 720);
    g_signal_connect(main_window, "destroy", G_CALLBACK(on_main_window_destroy_gtk4), NULL);

    GtkWidget *gl_area_widget = gtk_gl_area_new();
    gtk_gl_area_set_has_depth_buffer(GTK_GL_AREA(gl_area_widget), TRUE);
    gtk_gl_area_set_allowed_apis(GTK_GL_AREA(gl_area_widget), GDK_GL_API_GL);
    g_signal_connect(gl_area_widget, "render", G_CALLBACK(on_rendered), NULL);
    g_signal_connect(gl_area_widget, "realize", G_CALLBACK(when_realised), NULL);

    /* Keyboard controllers — direct to input_control.c full handlers */
    GtkEventController *key_ctrl = gtk_event_controller_key_new();
    g_signal_connect(key_ctrl, "key-pressed", G_CALLBACK(on_keypress), &main_inputs);
    g_signal_connect(key_ctrl, "key-released", G_CALLBACK(on_key_released), &main_inputs);
    gtk_widget_add_controller(main_window, key_ctrl);

    GtkEventController *motion_ctrl = gtk_event_controller_motion_new();
    g_signal_connect(motion_ctrl, "motion", G_CALLBACK(on_mouse_movements), &main_inputs);
    gtk_widget_add_controller(main_window, motion_ctrl);

    GtkGesture *click = gtk_gesture_click_new();
    gtk_gesture_single_set_button(GTK_GESTURE_SINGLE(click), 0);
    g_signal_connect(click, "pressed", G_CALLBACK(on_button_press), &main_inputs);
    g_signal_connect(click, "released", G_CALLBACK(on_button_release), &main_inputs);
    gtk_widget_add_controller(main_window, GTK_EVENT_CONTROLLER(click));

    GtkEventController *focus_ctrl = gtk_event_controller_focus_new();
    g_signal_connect(focus_ctrl, "leave", G_CALLBACK(on_focus_out), &main_inputs);
    gtk_widget_add_controller(main_window, focus_ctrl);

    GtkWidget *ui_overlay_layout = overlay_initialise(gl_area_widget);
    gtk_window_set_child(GTK_WINDOW(main_window), ui_overlay_layout);

    gtk_widget_set_can_focus(main_window, TRUE);
    gtk_widget_grab_focus(main_window);
    physics_timeout_id = g_timeout_add(16, physics_step_increment, gl_area_widget);
    frame_timer_init(&main_timer);
    gtk_window_present(GTK_WINDOW(main_window));
}

/* Keep old main_algorithm for source compat but route through GApplication. */
int main_algorithm(int argc, char *argv[]) {
    g_setenv("GDK_BACKEND", "x11", TRUE);
    mpe_app = gtk_application_new("org.mpe.engine", G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect(mpe_app, "activate", G_CALLBACK(app_activate), NULL);
    int status = g_application_run(G_APPLICATION(mpe_app), argc, argv);
    g_object_unref(mpe_app);
    mpe_app = NULL;
    mpe_config_save("status/engine.cfg");
    printf("[config] saved status/engine.cfg\n");
    return status;
}
int main(int argc, char *argv[]) {
    return main_algorithm(argc, argv);
}

#else /* ====================== GTK3 PATH (unchanged) ====================== */
static void on_main_window_destroy(GtkWidget *widget, gpointer user_data) {
    (void) widget;
    (void) user_data;
    if (physics_timeout_id) {
        g_source_remove(physics_timeout_id);
        physics_timeout_id = 0;
    }
    render_cleanup();
    physics_world_cleanup(physics_world_get_primary());
    gtk_main_quit();
}
//On Call
static void when_realised_GTK3(GtkGLArea *gl_area_widget) {
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
static gboolean on_rendered_GTK3(GtkGLArea *gl_area_widget, GdkGLContext *gl_context_data) {
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
    /* Primary simulation world owns all sim state (bodies, joints,
     * caches, scratch). Scene code assumes it initialized. */
    physics_world_init(physics_world_get_primary());
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
    g_signal_connect(gl_area_widget, "render", G_CALLBACK(on_rendered_GTK3), NULL);
    g_signal_connect(gl_area_widget, "realize", G_CALLBACK(when_realised_GTK3), NULL);
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
#endif /* MPE_GTK4 / GTK3 */
