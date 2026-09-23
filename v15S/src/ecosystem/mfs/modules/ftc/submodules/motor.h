/* MPE_FTC_070: DC motor electrical model */
#ifndef motor_h
#define motor_h

typedef struct {
    /* Electrical (derive from spec sheet: stall_torque, free_speed, stall_current) */
    float resistance; /* ohms */
    float kt; /* N·m/A torque constant */
    float kv; /* V/(rad/s) back-EMF constant */
    float stall_current; /* A */
    float free_speed_rad_s; /* rad/s at no load */

    /* Mechanical */
    float gear_ratio; /* output/input */
    float efficiency; /* 0..1 */

    /* Live state */
    float command; /* -1..1 from controller */
    float current; /* A (computed each tick) */
    float back_emf; /* V (computed each tick) */
    float torque; /* N·m at motor shaft */
    float output_torque; /* N·m at wheel after gearing */
    /* Explicit instantaneous torque at the measured speed (same endpoints
     * as output_torque, no observer softening): the STABLE torque actually
     * applied is output_torque (implicit+observer); this field sizes
     * feedforward force models (roller thrust) at true locked-rotor
     * physics instead of the observer's soft fixed point. */
    float torque_explicit;
    float rpm; /* current output speed */
    float temperature; /* simplified thermal model */
    /* Disturbance observer (implicit-load solve): external load torque
     * = measured net torque effect minus last tick's explicit motor
     * torque. Lets the implicit solve hold full stall torque against
     * locked wheels (prediction-error observers converge to a soft
     * fixed point ~6x low) while staying stable on free wheels. */
    float load_torque;
    float w_prev;
    float tau_exp_prev;
    int wprev_valid;
} motor;

/* Derive motor params from the four spec-sheet numbers. */
void motor_from_spec(motor *m, float stall_torque_nm, float free_speed_rpm, float stall_current_a,
                     float nominal_voltage, float gear_ratio, float efficiency);

/* Advance one tick. wheel_angular_vel = output shaft speed (rad/s). */
void motor_update(motor *m, float wheel_angular_vel, float dt, float battery_voltage);
/* Implicit-in-speed variant: solves back-EMF equilibrium at end-of-tick
 * speed, so light wheels cannot relaxation-oscillate around free speed
 * (explicit Euler moves ~30 rad/s per tick at stall torque vs a 2.5e-4
 * inertia — unconditionally unstable without this). Same spec endpoints
 * (stall/free); only the transient is stabilized. axle_inertia <= 0
 * falls back to the explicit update. */
void motor_update_load(motor *m, float wheel_angular_vel, float dt, float battery_voltage,
                       float axle_inertia);

#endif /* motor_h */
