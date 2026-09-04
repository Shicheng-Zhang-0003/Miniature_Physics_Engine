/* MPE_FTC_072: Battery model with voltage sag */
#ifndef battery_h
#define battery_h

typedef struct {
    float nominal_voltage; /* V (12.8 fresh) */
    float internal_resistance; /* ohms (~0.05 for FTC battery) */
    float capacity_ah; /* amp-hours */
    float charge_fraction; /* 0..1 */
} battery;

void battery_init(battery *b);
/* Returns terminal voltage under load. total_current = sum of all motor currents. */
float battery_get_voltage(const battery *b, float total_current_draw);
/* Drain battery over time based on current draw. */
void battery_drain(battery *b, float total_current_draw, float dt);

#endif /* battery_h */
