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
     * applied is output_torque (implicit+observer); this field is the
     * observer's reference (tau_exp_prev: external load = measured net
     * effect minus this). DESPOT-2026-09-28: the old comment claimed it
     * "sizes feedforward force models (roller thrust)" — no such consumer
     * exists (the analytic lateral is purely slip-velocity-driven; the
     * retired sin45*sum(tau)/r term is gone). */
    float torque_explicit;
    float rpm; /* current output speed */
    float temperature; /* simplified thermal model */
    /* Disturbance observer (implicit-load solve): external load torque
     * = measured net torque effect minus last tick's explicit motor
     * torque. Lets the implicit solve hold near-full stall torque against
     * locked wheels while staying stable on free wheels.
     * DESPOT-2026-09-29: the old comment here claimed prediction-error
     * observers "converge to a soft fixed point ~6x low". That was retracted
     * against the gated `mfs_t_stall_endpoint`, which measured 16% (3.1279
     * N.m against a 3.7265 N.m spec), not 6x.
     *
     * DESPOT-2026-10-03: AND THE 16% IS THERMAL TOO, NOT THE OBSERVER. Two
     * successive misattributions of one number, both now settled by direct
     * measurement (same rig, observer armed identically, copper temperature
     * the only variable):
     *     thermal ACTIVE         -> 3.12793 N.m  (-16.062%)
     *     temperature pinned 25C -> 3.72650 N.m  ( +0.000%)
     * The observer contributes EXACTLY NOTHING to the softening. The mechanism
     * is the copper model below, r_eff = R*(1 + 0.00393*(T - 25)), which
     * reaches r_eff/R = 1.19237 at T = 73.95 C after 300 stall ticks; and
     * 1/1.19237 = 0.8387, i.e. -16.1%. A motor held at 25 C delivers the full
     * spec stall torque through the observer without difficulty.
     *
     * CONSEQUENCE FOR ANYONE READING A 16% STALL SHORTFALL: check the motor
     * temperature before suspecting the estimator. `mfs_t_stall_endpoint` now
     * gates BOTH paths -- the derated value against the r_eff(T) model, and
     * the 25 C value against spec at 2% -- so the two are separated by test
     * rather than by argument. The genuinely-open observer-coupling defect is
     * a DIFFERENT one (large-load back-EMF misread, tracked as [MOTOR-III] in
     * docs/KNOWN_FAILURES.md); do not conflate them with this. */
    /* Set ONLY by motor_observe(), in the same module that consumes it
     * (DESPOT-2026-09-29). Callers must not touch wprev_valid/load_torque/
     * w_prev directly: they used to, which meant a caller that forgot left a
     * silently dead observer (tau_L == 0) with no diagnostic. */
    float load_torque;
    float w_prev;
    float tau_exp_prev;
    int wprev_valid;
} motor;
/* Derive motor params from the four spec-sheet numbers. */
void motor_from_spec (motor *m, float stall_torque_nm, float free_speed_rpm, float stall_current_a,
                      float nominal_voltage, float gear_ratio, float efficiency);
/* Advance one tick. wheel_angular_vel = output shaft speed (rad/s). */
void motor_update (motor *m, float wheel_angular_vel, float dt, float battery_voltage);
/* Implicit-in-speed variant: solves back-EMF equilibrium at end-of-tick
 * speed, so light wheels cannot relaxation-oscillate around free speed
 * (explicit Euler moves ~160 rad/s per tick at stall torque vs a 2.5e-4
 * axle inertia — DESPOT-2026-09-28: was "~30", recomputed 2.38*(1/60)/
 * 2.5e-4 ~= 159 for 19.2:1, 250+ for 26.9:1 and up — unconditionally
 * unstable without this). Same spec endpoints (stall/free); only the
 * transient is stabilized. axle_inertia <= 0 falls back to explicit. */
void motor_update_load (motor *m, float wheel_angular_vel, float dt, float battery_voltage, float axle_inertia);
/* DESPOT-FIX: defined in motor.c but never declared — every caller took an
 * implicit declaration (works by ABI luck, breaks under -Werror). */
void motor_reset_observer (motor *m);
/* Disturbance-observer update: estimate the external load on the wheel from
 * the measured shaft acceleration and last tick's explicit motor torque, and
 * publish it (with its validity flag) for motor_update_load() to consume.
 * Call once per wheel per tick, BEFORE motor_update_load(). Safe to call on
 * every consumer path: it cannot be called wrong into a dead observer.
 * DESPOT-2026-09-29. */
void motor_observe (motor *m, float wheel_angular_vel, float dt, float axle_inertia);
#endif /* motor_h */
