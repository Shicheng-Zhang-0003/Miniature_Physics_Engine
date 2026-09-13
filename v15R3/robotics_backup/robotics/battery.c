/* MPE_FTC_072: Battery model implementation */
#include "battery.h"

void battery_init(battery *b) {
    if (!b) {
        return;
    }
    b->nominal_voltage = 12.8f;
    b->internal_resistance = 0.015f; /* FIX 112: realistic LiPo internal resistance */
    /* FIX-AUDIT: 30Ah was 5-10x a real FTC pack (REV slim 3Ah, goBILDA 6Ah).
     * 5Ah gives realistic brown-out over a match. */
    b->capacity_ah = 5.0f;
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
    float open_circuit = b->nominal_voltage - 0.8f * (1.0f - soc) * (1.0f - soc);
    float sag = b->internal_resistance * total_current_draw;
    float terminal = open_circuit - sag;
    if (terminal < 0.0f) {
        terminal = 0.0f;
    }
    return terminal;
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
