/* MPE_FTC_072: Battery model with voltage sag */
#ifndef battery_h
#define battery_h

/* Pack fuse: 20 A PTC, resettable (both standard FTC packs carry one).
 * fuse_heat integrates sustained overcurrent; the pack browns out while
 * tripped and recovers when the load drops. */
#define MPE_BATTERY_FUSE_A 20.0f
#define MPE_BATTERY_FUSE_RESET_A 15.0f
#define MPE_BATTERY_OCV_FLOOR_V 10.0f /* NiMH 10-cell cutoff */

typedef struct {
    float nominal_voltage; /* V (12.8 fresh) */
    float internal_resistance; /* ohms (0.015 LiPo; see battery.c FIX 112) */
    float capacity_ah; /* amp-hours */
    float charge_fraction; /* 0..1 */
    float fuse_heat; /* 0..1+ ; >=1 tripped until load drops */
} battery;

void battery_init(battery *b);
/* Returns terminal voltage under load. total_current = sum of all motor currents. */
float battery_get_voltage(const battery *b, float total_current_draw);
/* Drain battery over time based on current draw. */
void battery_drain(battery *b, float total_current_draw, float dt);
/* Fuse thermal step: call once per tick before reading voltage. */
void battery_fuse_step(battery *b, float total_current_draw, float dt);
int battery_fuse_tripped(const battery *b);
void battery_reset_fuse(battery *b);

#endif /* battery_h */
