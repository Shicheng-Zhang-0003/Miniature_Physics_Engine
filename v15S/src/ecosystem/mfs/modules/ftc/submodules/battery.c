/* MPE_FTC_072: Battery model implementation */
#include "battery.h"
#include <math.h>
#include <stdio.h>

void battery_init(battery *b) {
    if (!b) {
        return;
    }
    b->fuse_heat = 0.0f;
    b->nominal_voltage = 12.8f;
    /* FIX-AUDIT-DESPOT: was 0.015 ohm (LiPo-class, and the header still said
     * "0.015 LiPo"). Both legal FTC packs are 10-cell NiMH (REV-31-1302,
     * goBILDA 3100-0012-0020): ~6 mohm/cell x 10 cells + wiring/connector
     * losses ~= 0.06 ohm pack-level. 0.015 hid ~4x of real sag (a 20 A
     * breakaway sagged 0.3 V instead of 1.2 V), masking brownout. */
    b->internal_resistance = 0.06f;
    /* VERIFIED: both standard FTC packs are 10-cell 12 V NiMH 3000 mAh
     * with 20 A fuses (REV-31-1302 slim, 567 g; goBILDA 3100-0012-0020).
     * 3.0 Ah exactly — the old 5.0 Ah compromise is retired. Nominal
     * 12.8 V is a fresh-pack assumption (spec nominal is 12.0 V). */
    b->capacity_ah = 3.0f;
    b->charge_fraction = 1.0f;
}

float battery_get_voltage(const battery *b, float total_current_draw) {
    /* FIX-AUDIT-DESPOT: was a silent 12.8 V default on NULL, which made a
     * missing-battery bug read as a fresh pack. NULL is a caller bug: warn
     * and return NAN so it propagates visibly instead of driving motors.
     * All in-tree callers pass live batteries (checked). */
    if (!b) {
        fprintf(stderr, "battery_get_voltage: NULL battery (caller bug)\n");
        return NAN;
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
     * the 10 V pack cutoff (was 8.8 V: below any real cutoff).
     * FIX-AUDIT-DESPOT: the quadratic shoulder below is a NOMINAL curve
     * fit, not a measured OCV table — labelled as such so nobody cites it
     * as cell data. */
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
    /* DESPOT-FIX: NaN/Inf current used to poison fuse_heat forever (NaN
     * comparisons false, heat stuck). Reject non-finite loads. */
    if (!isfinite(total_current_draw)) return;
    if (!isfinite(b->fuse_heat)) b->fuse_heat = 0.0f;
    float over = total_current_draw - MPE_BATTERY_FUSE_A;
    if (over > 0.0f) {
        /* ~20 A·s above rating trips (brief breakaway transients ride
         * through; sustained stall trips). */
        b->fuse_heat += over * dt / 20.0f;
        if (b->fuse_heat > 10.0f) b->fuse_heat = 10.0f; /* cap: flag, not accumulator */
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
    /* DESPOT-FIX: NaN current drained SoC to NaN and bricked the pack for the
     * run; negative spikes over-charged past physics. Reject non-finite,
     * allow negative (regen, credited upstream at 50%) but keep SoC in
     * [0,1] — overcharge is clamped, never stored. */
    if (!isfinite(total_current_draw)) return;
    if (!isfinite(b->charge_fraction)) b->charge_fraction = 1.0f;
    float amp_hours_used = (total_current_draw * dt) / 3600.0f;
    b->charge_fraction -= amp_hours_used / b->capacity_ah;
    if (b->charge_fraction < 0.0f) {
        b->charge_fraction = 0.0f;
    }
    if (b->charge_fraction > 1.0f) {
        b->charge_fraction = 1.0f;
    }
}
