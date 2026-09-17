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
    float rpm; /* current output speed */
    float temperature; /* simplified thermal model */
} motor;

/* Derive motor params from the four spec-sheet numbers. */
void motor_from_spec(motor *m, float stall_torque_nm, float free_speed_rpm, float stall_current_a,
                     float nominal_voltage, float gear_ratio, float efficiency);

/* Advance one tick. wheel_angular_vel = output shaft speed (rad/s). */
void motor_update(motor *m, float wheel_angular_vel, float dt, float battery_voltage);

#endif /* motor_h */
