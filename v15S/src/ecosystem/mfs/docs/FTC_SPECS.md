# FTC True Tech Specs — sourced reference for the MPE motor presets

Every number below is traceable to a manufacturer publication. "Published"
means printed on the vendor's product/spec page; "ideal-derived" means
computed as base-motor spec × gear ratio (marked as such); "nominal" means
an engineering estimate, also marked. **No physics here is invented.**

Conversions used: 1 kg.cm = 0.0980665 N·m · 1 oz-in = 0.00706155 N·m ·
1 ft-lb = 1.35582 N·m · all specs at nominal 12 VDC unless noted.

Model note: `motor_from_spec()` derives Kt from the published OUTPUT
stall (`Kt = stall/(gear*eff)/Istall`), so simulated stall output equals
the published stall exactly at stall current; efficiency therefore
cancels and is kept only as a nominal build marker (0.85 planetary /
0.80 spur). Gear ratio is used for encoder math.

## goBILDA 5203 / 5204 Yellow Jacket planetary (RS-555 base)

Base: RS-555 brushed DC, 12 VDC, 0.25 A no-load, 9.2 A stall, steel
planetary, 28 PPR magnetic encoder (×ratio at output). 5204 = same
gearboxes with 80 mm shaft (identical physics — no separate presets).
Source: goBILDA 5203/5204 product pages + spec sheets.

| SKU | Ratio | Free RPM | Stall (kg.cm) | Stall (N·m) | Preset |
|---|---|---|---|---|---|
| 5203-2402-0001 | 1:1 | 6000 | 1.47 | 0.1442 | — (base reference) |
| 5203-2402-0003 | 3.7:1 | 1620 | 5.4 | 0.5296 | — |
| 5203-2402-0005 | 5.2:1 | 1150 | 7.9 | 0.7747 | `MOTOR_GB_5203_5_2` |
| 5203-2402-0014 | 13.7:1 | 435 | 18.7 | 1.8338 | — |
| 5203-2402-0019 | 19.2:1 | 312 | 24.3 | 2.3830 | `MOTOR_GB_5203_19_2` |
| 5203-2402-0027 | 26.9:1 | 223 | 38.0 | 3.7265 | `MOTOR_GB_5203_26_9` |
| 5203-2402-0051 | 50.9:1 | 117 | 68.4 | 6.7077 | `MOTOR_GB_5203_50_9` |
| 5203-2402-0071 | 71.2:1 | 84 | 93.6 | 9.1790 | `MOTOR_GB_5203_71_2` |
| 5203-2402-0100 | 99.5:1 | 60 | 133.2 | 13.0625 | `MOTOR_GB_5203_99_5` |
| 5203-2402-0139 | 139:1 | 43 | 185 | 18.1423 | — |
| 5203-2402-0188 | 188:1 | 30 | 250 | 24.5166 | `MOTOR_GB_5203_188` |

Cross-check (efficiency is real, not assumed): base stall 1.47 kg.cm ×
19.2 = 28.2 ideal vs 24.3 published → 0.86 gearbox efficiency. The 0.85
nominal used in presets is consistent with published data.

5202 series (6 mm D-shaft) mirrors these ratios on the same RS-555/9.2 A
platform (e.g. 5202-2402-0019: 19.2:1, 312 RPM, 24.3 kg.cm) — same
physics as the 5203 equivalents, with 11 duplicate presets present in
motor_presets.h (MOTOR_GB_5202_*). 5303 Saturn
series is 24 V and NOT FTC-legal — excluded.

## AndyMark NeveRest (am-3104 base)

Base motor am-3104: 12 VDC, 6000 RPM ±10% no-load, 0.062 N·m stall,
11.5 A stall, 0.4 A free, 7 PPR encoder. Source: AndyMark am-3104 pages.
(All Classic/Orbital/Hex gearboxes accept this motor; all NeveRest
versions are FTC-legal per AndyMark.)

| Preset | Ratio | Free RPM | Stall (N·m) | Stall A | Basis |
|---|---|---|---|---|---|
| `MOTOR_NR_CLASSIC_40` | 40:1 | 160 pub | 2.4715 (350 oz-in pub) | 11.5 | published |
| `MOTOR_NR_CLASSIC_60` | 60:1 | 105 pub | 3.707 (2.734 ft-lb pub) | 11.5 | published |
| `MOTOR_NR_ORBITAL_19_2` | 19.2:1 | ~344 pub | 1.1904 (ideal-derived) | 11.5 | mixed |

Classic stalls match base×ratio almost exactly (0.062×40 = 2.48 ≈
2.472), i.e. AndyMark publishes ideal numbers. Orbital free speeds run
~10% above 6000/ratio (6600-class base behavior); published ~344 used.
Orbital 3.7 (~1784 RPM) and 50.9 (~130 RPM) presets exist
(`MOTOR_NR_ORBITAL_3_7`, `MOTOR_NR_ORBITAL_50_9`).

## REV Robotics

| Preset | Spec | Free RPM | Stall (N·m) | Stall A | Basis |
|---|---|---|---|---|---|
| `MOTOR_REV_CORE_HEX` | REV-41-1300, 72:1 | 125 pub | 3.2 pub | 4.4 pub | published |
| `MOTOR_REV_HD_HEX` | REV-41-1291 bare | 6000 pub | 0.105 pub | 8.5 pub | published |
| `MOTOR_REV_HD_HEX_20` | 20:1 spur | 300 pub | 2.10 ideal-derived | 8.5 | derived |
| `MOTOR_REV_HD_HEX_40` | 40:1 spur | 150 pub | 4.20 ideal-derived | 8.5 | derived |
| `MOTOR_REV_UP_12` | UltraPlanetary 3×4 | 500 nom | 1.26 ideal-derived | 8.5 | derived |
| `MOTOR_REV_UP_60` | UltraPlanetary 3×4×5 | 100 nom | 6.30 ideal-derived | 8.5 | derived |

Sources: REV DUO docs (Core Hex page; HD Hex page: base 6000 RPM /
0.105 N-m / 8.5 A / 0.4 A no-load / 28 PPR; encoder doc: 40:1→150 RPM,
20:1→300 RPM; UltraPlanetary kit: 3/4/5 cartridges, stacks 3:1–60:1).
Geared HD/UP stall torques are NOT published by REV — ideal-derived
(base × ratio), marked as such. NEO brushless is FRC-only, excluded.

## Pitsco TETRIX TorqueNADO (spur, 324 g, 6 mm D, 8.7 A stall)

Source: Pitsco spec PDFs (both agree).

| Preset | Ratio | Free RPM | Stall (oz-in) | Stall (N·m) |
|---|---|---|---|---|
| `MOTOR_TN_20` | 20:1 | 300 pub | 233 | 1.6453 |
| `MOTOR_TN_40` | 40:1 | 150 pub | 466 | 3.2907 |
| `MOTOR_TN_60` | 60:1 | 100 pub | 700 | 4.9431 |

Self-consistent line: 4.9431/60×20 = 1.6477 ≈ 1.6453 ✓. Encoder:
6 cycles (24 counts) per motor rev; 1440 counts/output rev at 60:1.

## Batteries (FTC legal packs — both 12 V NiMH 3000 mAh, 20 A fuse)

- REV-31-1302 slim: 10-cell NiMH, 12 V, 3000 mAh, 567 g, XT30,
  20 A ATM fuse (cells rated 10C/30 A, fuse-limited to 20 A).
- goBILDA 3100-0012-0020: 12 V NiMH, 3000 mAh, XT30, 20 A fuse,
  30 A max discharge.
- `battery_init`: 12.8 V nominal = fresh-pack assumption (spec nominal
  is 12.0 V); 0.015 Ω internal resistance nominal; **3.0 Ah exact**.
  The old 5.0 Ah compromise is retired.

## Wheels & chassis references

- goBILDA 96 mm mecanum set (3213-3606-0002): 207 g/wheel, 70A
  durometer bearing rollers. Strafer kit pairs 5203-2402-0019
  (19.2:1, 312 RPM) with 96 mm wheels for 1.57 m/s theoretical:
  312/60 rev/s × π × 0.096 m = 1.568 m/s ✓ (validates the pairing).
- GripForce 104 mm mecanum set also exists. AndyMark 4 in. mecanums exist.
- Sim defaults (r = 0.05 m, 100 mm class) sit between these; fine.
- FTC robot weight limit is long-standing 42 lb (19.05 kg) — verify
  against the current season's Game Manual. Sim robot ≈ 8.8 kg total,
  well inside it.

## What was wrong before (2026-09 audit)

The previous preset table mixed NeveRest-class RPMs with invented
torques/currents under goBILDA names (e.g. 340 RPM / 1.63 N-m / 17 A
matches NO published sheet; true 19.2:1 is 312 / 2.383 / 9.2; stall
17 A matches no FTC motor on record — the real figures are 9.2 A
goBILDA, 11.5 A NeveRest, 8.7 A TorqueNADO, 8.5/4.4 A REV). The
`MOTOR_GB_5203_30/43_7/71` identifiers named ratios that do not exist
in the 5203 line (real neighbors: 26.9, 50.9, 71.2) and were replaced.
`MOTOR_REV_CORE_HEX` was 60 RPM / 1.40 N-m / 10 A vs published
125 / 3.2 / 4.4. All entries above replace them.
