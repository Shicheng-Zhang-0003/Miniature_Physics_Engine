/* MPE_FTC_072: Battery model implementation */
#include "battery.h"

void battery_init(battery *b) {
    if (!b) {
        return;
    }
    b->fuse_heat = 0.0f;
    b->nominal_voltage = 12.8f;
    b->internal_resistance = 0.015f; /* FIX 112: realistic LiPo internal resistance */
    /* VERIFIED: both standard FTC packs are 10-cell 12 V NiMH 3000 mAh
     * with 20 A fuses (REV-31-1302 slim, 567 g; goBILDA 3100-0012-0020).
     * 3.0 Ah exactly — the old 5.0 Ah compromise is retired. Nominal
     * 12.8 V is a fresh-pack assumption (spec nominal is 12.0 V). */
    b->capacity_ah = 3.0f;
    b->charge_fraction = 1.0f;
}

float battery_get_voltage(const battery *b, float total_current_draw) {
    if (!b) {
        return 12.8f;
    }
    /* FIX-AUDIT: OCV was linear in SoC (6.4V at 50% - non-physical).
     * LiPo OCV is flat ~12.4-12.8V then cliff. Quadratic shoulder. */
    float soc = b->charge_fraction;
    if (soc < 0.0f) {
        soc = 0.0f;
    }
    if (soc > 1.0f) {
        soc = 1.0f;
    }
    /* PHYSICS-FIX: NiMH 10-cell swing is ~4 V fresh->cutoff (14 V fresh,
     * 12 V nominal, ~10 V cutoff), not 0.8 V. The old 0.8 V shoulder kept
     * a dead pack at 12.0 V OCV, hiding brownout in long runs. Floored at
     * the 10 V pack cutoff (was 8.8 V: below any real cutoff). */
    float open_circuit = b->nominal_voltage - 4.0f * (1.0f - soc) * (1.0f - soc);
    if (open_circuit < MPE_BATTERY_OCV_FLOOR_V) {
        open_circuit = MPE_BATTERY_OCV_FLOOR_V;
    }
    /* Tripped PTC passes brownout voltage, not zero (resettable fuse). */
    if (b->fuse_heat >= 1.0f) {
        return 1.2f;
    }
    float sag = b->internal_resistance * total_current_draw;
    float terminal = open_circuit - sag;
    if (terminal < 0.0f) {
        terminal = 0.0f;
    }
    return terminal;
}

void battery_fuse_step(battery *b, float total_current_draw, float dt) {
    if (!b || dt <= 0.0f) return;
    float over = total_current_draw - MPE_BATTERY_FUSE_A;
    if (over > 0.0f) {
        /* ~20 A·s above rating trips (brief breakaway transients ride
         * through; sustained stall trips). */
        b->fuse_heat += over * dt / 20.0f;
    } else if (total_current_draw < MPE_BATTERY_FUSE_RESET_A) {
        b->fuse_heat -= dt / 5.0f;
        if (b->fuse_heat < 0.0f) b->fuse_heat = 0.0f;
    }
}

int battery_fuse_tripped(const battery *b) {
    return (b && b->fuse_heat >= 1.0f) ? 1 : 0;
}

void battery_reset_fuse(battery *b) {
    if (b) b->fuse_heat = 0.0f;
}

void battery_drain(battery *b, float total_current_draw, float dt) {
    if ((!b) || (dt <= 0.0f)) {
        return;
    }
    float amp_hours_used = (total_current_draw * dt) / 3600.0f;
    b->charge_fraction -= amp_hours_used / b->capacity_ah;
    if (b->charge_fraction < 0.0f) {
        b->charge_fraction = 0.0f;
    }
    if (b->charge_fraction > 1.0f) {
        b->charge_fraction = 1.0f;
    }
}
