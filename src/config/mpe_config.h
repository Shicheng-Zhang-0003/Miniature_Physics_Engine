/* MPE_TASK_26_CONFIG_API_BEGIN */
#ifndef mpe_config_h
#define mpe_config_h

#include <stdbool.h>
#include <stddef.h>

/* ==================================================================
 * MPE Tunable Configuration Store
 *
 * This header defines:
 * 1. mpe_config_t — the live struct holding all tunable values
 * 2. mpe_param — metadata describing each tunable (for menu/terminal)
 * 3. The registry API for init/get/set/save/load
 *
 * Hot-path code reads g_cfg.<group>.<field> directly.
 * Cold-path code (menu, terminal, save/load) iterates the registry.
 * ================================================================== */

/* ------------------------------------------------------------------
 * Parameter metadata enums
 * ------------------------------------------------------------------ */
typedef enum {
    P_FLOAT,
    P_INT,
    P_BOOL
} param_type;

typedef enum {
    CAT_WORLD,
    CAT_TIMESTEP,
    CAT_SLEEP,
    CAT_SOLVER,
    CAT_DEPENETRATION,
    CAT_BROADPHASE,
    CAT_JOINTS,
    CAT_BOUNDARY,
    CAT_SPAWNER,
    CAT_BODY_DEFAULTS,
    CAT_CAMERA,
    CAT_RENDER,
    CAT_UI
} param_category;

/* ------------------------------------------------------------------
 * The live config struct — all tunables grouped by domain
 * ------------------------------------------------------------------ */
typedef struct {
    struct {
        float gravity;
        float drag;
        float floor_friction_s;
        float floor_friction_k;
    } world;

    struct {
        int solver_iterations;
        int max_substeps;
        float max_linear_speed;
        float max_angular_speed;
    } timestep;

    struct {
        float linear_thresh_sq;
        float angular_thresh_sq;
        float timer_duration;
        float wake_linear_thresh_sq;
        float wake_angular_thresh_sq;
    } sleep;

    struct {
        float penetration_slop;
        float bias_factor;
        float max_separation_bias;
        float restitution_velocity_thresh;
        float max_restitution_bias;
        float static_friction_thresh;
        float warm_start_match_dist_sq;
    } solver;

    struct {
        float correction_factor;
        float max_correction;
        float penetration_slop;
        float wake_depth_thresh;
        int rebuild_iterations;
    } depenetration;

    struct {
        float cell_size_default;
        float cell_size_min;
        float cell_size_max;
        float cell_size_multiplier;
        int max_cell_span_per_axis;
    } broadphase;

    struct {
        float max_acceleration;
        float default_spring_k;
        float default_damping;
        float soft_spring_k;
        float soft_damping;
    } joints;

    struct {
        float floor_emergency_slop;
        float floor_velocity_slop;
    } boundary;

    struct {
        float mass;
        float radius;
        float cube_mass;
        float cube_extent;
        float speed;
        float friction_s;
        float friction_k;
        int overlap_max_attempts;
        float overlap_thresh;
    } spawner;

    struct {
        float sphere_restitution;
        float sphere_fric_s;
        float sphere_fric_k;
        float cube_restitution;
        float cube_fric_s;
        float cube_fric_k;
    } body_defaults;

    struct {
        float move_speed;
        float mouse_sensitivity;
        float steer_sensitivity;
        float horizontal_friction;
        float jump_height;
        float ijkl_speed;
    } camera;

    struct {
        float light_x, light_y, light_z;
        float ambient_strength;
        float specular_coeff;
        float specular_exponent;
    } render;

    struct {
        float change_rate_game;
        float change_rate_debug;
        int long_run_ticks;
        float enter_spawn_delay;
        float enter_spawn_interval;
    } ui;
} mpe_config_t;

/* ------------------------------------------------------------------
 * Parameter registry entry — describes one tunable
 * ------------------------------------------------------------------ */
typedef struct {
    const char     *key;          /* "solver.iterations" — file/terminal key */
    const char     *display;      /* "Solver Iterations" — menu label       */
    const char     *help;         /* tooltip / man text                     */
    param_type      type;
    param_category  category;
    void           *storage;      /* pointer into g_cfg                     */
    double          def;          /* default value                          */
    double          min;          /* clamp lower bound                      */
    double          max;          /* clamp upper bound                      */
    bool            debug_only;   /* requires debug mode to mutate          */
} mpe_param;

/* ------------------------------------------------------------------
 * The global config instance — hot path reads this directly
 * ------------------------------------------------------------------ */
extern mpe_config_t g_cfg;

/* ------------------------------------------------------------------
 * The registry table — cold path iterates this
 * Declared in mpe_config_schema.c, extern'd here for iteration.
 * ------------------------------------------------------------------ */
extern const mpe_param *g_registry;
extern const size_t g_registry_count;

/* ------------------------------------------------------------------
 * Registry API
 * ------------------------------------------------------------------ */

/* Initialize g_cfg to compile-time defaults. Call once at startup. */
void mpe_config_init(void);

/* Reset all tunables to their defaults. */
void mpe_config_reset_defaults(void);

/* Generic getters — look up by key string. Return false if not found. */
bool mpe_config_get_float(const char *key, float *out);
bool mpe_config_get_int(const char *key, int *out);
bool mpe_config_get_bool(const char *key, bool *out);

/* Generic setters — look up by key, clamp to [min,max], write to g_cfg.
 * Returns false if key not found or value was clamped. */
bool mpe_config_set_float(const char *key, float value);
bool mpe_config_set_int(const char *key, int value);
bool mpe_config_set_bool(const char *key, bool value);

/* Serialization — text INI format to/from disk.
 * load: missing keys keep defaults; unknown keys ignored; returns false on IO error.
 * save: writes all params grouped by category. */
bool mpe_config_load(const char *path);
bool mpe_config_save(const char *path);

/* Iteration helpers for menu/terminal. */
size_t mpe_config_count_by_category(param_category cat);
size_t mpe_config_get_by_category(param_category cat, const mpe_param **out_params, size_t max_params);

/* Find a param by key. Returns NULL if not found. */
const mpe_param *mpe_config_find(const char *key);

/* Category name for display. */
const char *mpe_config_category_name(param_category cat);

/* MPE_TASK_39_FIX_BACKUP_DECL_BEGIN */
bool mpe_config_save_backup(const char *path);
bool mpe_config_load_backup(const char *path);
/* MPE_TASK_39_FIX_BACKUP_DECL_END */

#endif /* mpe_config_h */
/* MPE_TASK_26_CONFIG_API_END */
