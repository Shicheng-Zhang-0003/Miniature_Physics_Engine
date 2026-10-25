# MPE v15S — Release Notes

> **Historical context:** v15R3 remains the frozen physics/configuration baseline. v15S consolidates the MFS robotics ecosystem into the main source tree and extends the engine with FTC-style robot simulation, configurable shooting, trajectory prediction, multi-piece intake handling, and match-level piece processing.
>
> **Validation note:** Test figures quoted from earlier v15S testing are historical results, not a guarantee that every subsequent working-tree change has been independently validated.

- **Status:** Development release / integration milestone
- **Scope:** Robotics integration, shooting and aiming corrections, intake and multi-piece handling, physics-path integration, regression analysis, and supporting diagnostics

## Overview

v15S extends the Miniature Physics Engine with an integrated robotics simulation environment. The underlying engine remains a hand-written 3D rigid-body engine in C, with GTK/OpenGL rendering and a headless validation workflow. The MFS ecosystem provides the robotics-specific layer for robot control, game-piece handling, shooting, aiming, and match simulation.

## 1. Robotics Integration and Shooting Consistency

v15S brings the MFS robotics implementation into the primary source tree and continues the transition from independently evolving simulation paths toward a shared physics-step implementation.

The shooting subsystem exposes configurable launch parameters, including:

- Launch elevation
- Launch speed
- Backspin
- Minimum launch power

The aim solver, firing path, and trajectory preview have received targeted corrections intended to reduce discrepancies between predicted and actual shots. The work focuses on removing sources of inconsistent behaviour rather than masking them with arbitrary velocity or angle adjustments.

### Shooting and Aiming Findings

The investigation identified several sources of prediction error:

- The aim solver and live simulation did not apply aerodynamic effects consistently.
- Runtime launch-parameter changes could leave cached aim solutions stale.
- Power limits were not applied consistently across the gamepad, solver, and trajectory preview.
- Some firing paths used a fixed elevation while publishing a different configured elevation.
- Trajectory-preview caching did not account for every parameter that affects the launch state.

Targeted fixes were prepared for these discrepancies. Shooting behaviour has improved, but final validation must use the exact committed tree and exercise all relevant firing paths.

**Correctness principle:** The trajectory predictor, published aim state, and actual launch state should agree on the physical parameters used for a shot. Any approximation should be explicit and tested.

## 2. Intake and Multi-Piece Handling

The intake subsystem is a primary focus of correctness and regression testing.

The implementation supports carrying up to four pieces per robot, with ordinary balls and BIOBUZZ pollen/nectar pieces participating in the intake and shooting workflow. The audit identified several areas where single-piece tests were insufficient to establish reliable multi-piece behaviour.

### Areas of Correctness Work

- **Capture orientation:** The capture region should remain aligned with the robot as it turns.
- **Piece dimensions:** Capture eligibility should distinguish a piece's centre from its full extent when modelling a physical aperture.
- **Mixed-piece storage:** Arrangement must account for different piece radii.
- **Candidate selection:** Selection should not depend accidentally on registry order.
- **Possession integrity:** Carried counts and body IDs require validation.
- **Ejection state:** Releasing a piece should establish a well-defined physical state.
- **Registry consistency:** Physical existence, intake ownership, and match-scoring registration are distinct but coordinated responsibilities.

### Multi-Piece Geometry

The previous arrangement used spacing derived from each piece individually and did not guarantee adequate separation for mixed sizes. Four nectar pieces also exceeded the declared intake-mouth lateral half-width under the previous single-row arrangement.

Corrective work introduces a group-layout approach and a dedicated geometry-invariant test. The intended guarantees are non-overlapping carried positions and compliance with the defined storage envelope. These guarantees should be verified against the simulation's coordinate conventions, collision representation, and intended robot geometry.

### Physical Ownership Versus Scoring

Ordinary-ball and BIOBUZZ paths differ in registration semantics. A piece may exist physically and be capturable without being registered as a match-scoring piece. Match-staged pieces carry metadata used by BIOBUZZ scoring.

Registry consolidation must not cause duplicate physics updates, duplicate capture, lost pieces, or incorrect scoring.

## 3. Physics-Path Integration

The v15S working tree includes changes to the simulation physics loop intended to route stepping through the shared physics-world implementation. Shared stepping can reduce the risk of divergent pipelines, but requires regression checks against the prior GUI path.

Compare positions, linear and angular velocities, aerodynamic effects, contacts, chassis movement, carried IDs and counts, and shooting/ejection transitions. A successful build alone does not prove behavioural equivalence.

## 4. Validation and Regression Strategy

### Existing Validation Context

An earlier v15S headless test run reported **32/32 tests passing** after stale callback signatures were temporarily corrected in a disposable test copy. This is a historical result, not proof that the original test source builds unchanged or that subsequent changes pass. The callback mismatch must be resolved in source before claiming a clean, reproducible test run.

### Shooting Regression Requirements

- Compare predicted and actual trajectories using identical parameters.
- Test minimum, intermediate, and maximum power.
- Change launch speed and spin without moving the robot to exercise cache invalidation.
- Verify that published elevation matches the actual launch elevation.
- Exercise keyboard, gamepad, solver-driven, and direct API paths.
- Test ordinary balls and BIOBUZZ pieces.

### Intake Regression Requirements

- Capture and carry pollen and nectar independently.
- Test four-pollen and four-nectar loads.
- Test mixed loads.
- Verify pairwise clearance and storage bounds.
- Test capture at multiple robot orientations.
- Test a full intake.
- Verify each piece is shot exactly once.
- Refill after shooting.
- Test stale IDs.
- Verify ejection linear and angular velocity.
- Verify match-staged pieces retain scoring metadata.

### Regression Policy

Test possession state and invariants, not merely finite simulation values or whether at least one piece was captured. Expected game rules should be explicit.

## 5. Configuration and Launch-State Consistency

Runtime launch configuration affects aiming, shooting, and preview. The following behaviours require verification:

- The configured minimum power is respected in firing and preview.
- Solver candidates remain within the configured legal range.
- Changes to speed, spin, and power limits invalidate cached results.
- Displayed aim matches the actual launch.
- Default and fallback APIs do not silently diverge.

Validate configuration at the boundary rather than relying on independent hard-coded clamps in separate subsystems.

## 6. Known Limitations and Remaining Verification

The following items remain verification requirements:

- Four-piece storage geometry must be checked for every relevant combination of piece sizes.
- Mixed-registry selection and ownership must be deterministic.
- Arcade-created and match-scoring pieces have different registration semantics.
- Physics-path equivalence needs comparison against the previous GUI path.
- End-to-end shooting and intake validation is still required.
- Test callback signatures and build dependencies need to be resolved for reproducible validation.
- The exact final working tree must be rebuilt and tested before release claims are made.

These are follow-up requirements, not claims that the systems are unusable.

## 7. Release Validation Checklist

Run the release ritual from the exact tree being committed:

```bash
cd v15S/src
make clean
make
python3 ../../tools/test_runner.py
make tui-smoke
```

Then launch the engine and run the in-engine validation matrix.

For robotics-specific validation:

1. Build the MFS plugin and test suite.
2. Run the full MFS suite without temporary source modifications.
3. Execute shooting and multi-piece intake regression scenarios.
4. Run `git diff --check` and inspect the complete working-tree diff.
5. Ensure generated binaries and temporary diagnostics are not unintentionally included.
6. Record the actual results. Do not reuse historical test counts without rerunning the tests.

## 8. Release Summary

v15S advances the engine from a physics-focused release toward integrated robotics simulation. Its central correctness priorities are consistency across the physics world, aim solver, firing APIs, trajectory preview, intake possession, carried-piece geometry, and match-scoring registries.

The shooting investigation exposed risks caused by independently evolving subsystems. The intake audit revealed related risks in mixed-piece handling, candidate selection, geometry, and object registration. This work establishes a clearer path toward reliable robotics simulation while documenting remaining validation requirements rather than concealing them behind successful compilation or isolated tests.
