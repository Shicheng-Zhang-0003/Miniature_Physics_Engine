/* MPE_FTC_072: Battery model with voltage sag */
#ifndef battery_h
#define battery_h
/* Pack fuse: 20 A. FIX-AUDIT 2026-11-09: the claim "resettable PTC" was
 * too strong — the real REV-31-1302 / goBILDA 3100-0012-0020 packs ship a
 * ONE-SHOT 20 A ATM blade fuse (blown = replace the fuse, match over). The
 * sim models a resettable PTC anyway (browns out at 1.2 V while tripped,
 * recovers when the load drops) so a tripped pack is recoverable
 * mid-match; treat the RECOVERY as a model choice, not a pack spec.
 * fuse_heat integrates sustained overcurrent. */
#define MPE_BATTERY_FUSE_A 20.0f
#define MPE_BATTERY_FUSE_RESET_A 15.0f
#define MPE_BATTERY_OCV_FLOOR_V 10.0f /* NiMH 10-cell cutoff */
typedef struct {
    float nominal_voltage; /* V (12.8 fresh) */
    float internal_resistance; /* ohms (0.06 NiMH pack-level; see battery.c FIX-AUDIT-DESPOT) */
    float capacity_ah; /* amp-hours */
    float charge_fraction; /* 0..1 */
    float fuse_heat; /* 0..10 (capped as a flag, battery.c: "flag, not
                      * accumulator"); >=1 tripped until load drops */
} battery;
void battery_init (battery *b);
/* Returns terminal voltage under load. total_current = sum of all motor currents. */
float battery_get_voltage (const battery *b, float total_current_draw);
/* Drain battery over time based on current draw. */
void battery_drain (battery *b, float total_current_draw, float dt);
/* Fuse thermal step: call once per tick before reading voltage. */
void battery_fuse_step (battery *b, float total_current_draw, float dt);
int battery_fuse_tripped (const battery *b);
void battery_reset_fuse (battery *b);
#endif /* battery_h */
