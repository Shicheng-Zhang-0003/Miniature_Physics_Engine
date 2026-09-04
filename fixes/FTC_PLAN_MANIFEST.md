# 475_FTC Simulator — Development Plan & Manifest

**Last Updated:** 2026-08-29
**Current Status:** 🟢 **GREEN** (all 8 headless tests passing, 0 build failures)
**Active Phase:** Milestone 3 (Solver Hardening)

---

## The Physics-First Pivot

The original development plan was "architecture-up": build the FTC API, sensors, and mechanisms on top of the existing physics engine.

**This plan was abandoned.**

Audit of the existing engine revealed that wheels were modeled as spheres, and mecanum strafing was faked via direct chassis forces (bypassing wheel friction). A simulator built on fake drivetrain physics is useless for real FTC autonomous tuning.

The new plan is **physics-first**. We must achieve physical correctness at the contact layer (cylinder wheels, anisotropic friction, iterative solving) *before* building the FTC API layer on top of it. The headless test harness is proven; now the physics must be made real.

---

## Milestone 1: Foundation & Robotics Core ✅ [COMPLETED]
*77 automated fix scripts applied. Headless test suite green.*

- [x] **Physics World Context:** Extracted global state into `physics_world` struct. Multi-world support proven (`two_world_test`).
- [x] **Constraint Framework:** Broadphase threading, constraint pools, and revolute joint solver (P2P + axis + motor).
- [x] **FTC Motor Model:** BackEMF, current, torque, and gearing pipeline. 5 goBILDA/REV presets.
- [x] **Battery Model:** Voltage sag under multi-motor load.
- [x] **Robot Object & Drivetrains:** Tank and Mecanum IK. (Note: Mecanum now uses real anisotropic roller friction on cylinder wheels.)
- [x] **Code Quality:** Clang-format normalization across 86 source files. Dead code removal.

---

## Milestone 2: The Physics Keystone ✅ [COMPLETED]
*Goal: Replace sphere wheels with cylinders and implement real anisotropic friction. This is the single most critical phase for simulator accuracy.*

- [x] **090: Cylinder Headers** — Add `object_cylinder` to enum, `cylinder_half_length` to struct, declare init/inertia functions.
- [x] **091: Cylinder Implementation** — Cylinder inertia tensor (X-axis axle), `physics_world_add_cylinder`, broadphase bounding radius.
- [x] **092: Cylinder–Floor Narrowphase** — Closest-point collision detection between cylinder rolling surface and ground plane.
- [x] **093: Cylinder–Object Narrowphase** *(cylinder-floor done; cylinder-vs-sphere/cube deferred)* — Cylinder vs. Sphere and Cylinder vs. OBB collisions.
- [x] **094: Anisotropic Friction Model** — Split tangential velocity into rolling (high grip) and axle (low slip) components. Add `roller_angle` for mecanum 45° rollers.
- [x] **095: Real Drivetrain Rebuild** — Rewrite `robot.c` to use cylinder wheels. **Delete** `wheel_traction.c` raycast hack and `drivetrain_mecanum` chassis-force cheat.
- [x] **096: Drivetrain Validation** — Tank drives straight, mecanum strafes via wheel contact, turning works, push-test resists lateral force.

---


### Post-Milestone-2 Fixes (097–128)
- [x] **100–106:** GUI robot bridge, proxy sync, spawn at (5,y,5)
- [x] **107–114:** Drive key mapping (G/V/B/N/C/H), motor gear ratio fix, battery tuning
- [x] **117:** Real traction (torque → ground force, clamped by friction)
- [x] **118–121:** Traction block repairs, config include fix
- [x] **122:** Physics pipeline fix (motor gearing, broadphase cylinder, fixed timestep)
- [x] **123–124:** Input held-state, velocity cap, damping balance
- [x] **125–125a:** Heading indicator (orange nose sphere)
- [x] **126–126a:** Revolute axis drift fix (Baumgarte β=0.2→0.1)
- [x] **127–127a:** Camera float fix, strafe diagnostics
- [x] **128–128a:** Strafe diagnostic scope fix

## Milestone 3: Solver Hardening ⬜ [PENDING]
*Goal: Make the constraint solver stiff and stable under the loads of a 4-wheel drivetrain.*

- [x] **100: Iterative Constraint Solve** *(constraint_solve_all moved inside solver iteration loop; warm-starting deferred)* — Move `revolute_solve` into the main solver iteration loop with accumulated impulses and warm-starting.
- [ ] **101: Per-World Contact Cache** — Move global contact cache arrays into the `physics_world` struct to ensure multi-world correctness.
- [ ] **102: Solver Islanding (PHYS-001)** — Group disjoint contact manifolds into islands to prevent unrelated contacts from consuming solver iterations.
- [ ] **103: Rolling Resistance (PHYS-004)** — Apply a small opposing torque to spinning wheels in contact with the floor to simulate realistic coast-down.

---

## Milestone 4: Sensors & Closed-Loop Control ⬜ [PENDING]
*Goal: Provide the data feedback loops required for FTC autonomous programming.*

- [ ] **110: Sensor Noise Framework** — Reusable Gaussian noise + drift model for all sensors.
- [ ] **111: Encoders** — Tick integration from wheel angular velocity. Velocity and position readings.
- [ ] **112: IMU** — Yaw/pitch/roll, angular velocity, linear acceleration. Model gyro drift over time.
- [ ] **113: Distance Sensors** — Raycast-based range finding (ultrasonic cone vs. laser beam).
- [ ] **114: PID Controller** — Reusable P/I/D struct with integral windup limits and output clamping.
- [ ] **115: Motor Control Modes** — Add Velocity (RPM) and Position (Ticks) modes alongside raw Power mode.
- [ ] **116: Sensor Validation Tests** — Drive known distances, rotate known angles, verify sensor outputs match physics within noise bounds.

---

## Milestone 5: Mechanisms & Kinematic Chains ⬜ [PENDING]
*Goal: Simulate arms, slides, and intakes.*

- [ ] **120: Prismatic (Slider) Joint** — Implement linear constraint solver (analogous to revolute).
- [ ] **121: Arm Mechanism** — Revolute joint + motor + angle limits + gravity compensation.
- [ ] **122: Linear Slide/Lift** — Prismatic joint + motor + extension limits.
- [ ] **123: Servos** — Position-controlled joints with realistic slew rates (no teleporting).
- [ ] **124: Intakes & Grabbing** — Touch sensors + temporary constraint welding for game elements.

---

## Milestone 6: Field Engine & Season Abstraction ⬜ [PENDING]
*Goal: Data-driven field generation so new FTC seasons don't require engine rewrites.*

- [ ] **130: Field Geometry Builder** — Construct 12x12ft fields from static OBB primitives (walls, tiles, ramps).
- [ ] **131: Field JSON Schema** — Declarative format for wall positions, tile layouts, and spawn points.
- [ ] **132: Game Elements** — Dynamic rigid bodies with season-specific mass/friction (rings, cones, pixels).
- [ ] **133: Scoring Zones** — Spatial triggers that detect when game elements enter defined regions.

---

## Milestone 7: FTC Platform & API ⬜ [PENDING]
*Goal: Allow teams to write standard FTC OpModes that run natively in the simulator.*

- [ ] **140: Robot JSON Loader** — Parse declarative robot definitions (chassis, wheels, mechanisms, sensors) into `ftc_robot` structs.
- [ ] **141: HardwareMap Abstraction** — `hardwareMap.getMotor()`, `getServo()`, `getIMU()` API layer.
- [ ] **142: OpMode Lifecycle** — `init()`, `init_loop()`, `start()`, `loop()`, `stop()` execution context.
- [ ] **143: Gamepad Model** — Dual gamepad state (sticks, triggers, buttons) fed from physical controllers.
- [ ] **144: Telemetry System** — Key-value data logging and display pipeline.

---

## Milestone 8: GUI Integration & Adoption ⬜ [PENDING]
*Goal: Bring the headless physics into the visual engine and prepare for distribution.*

- [ ] **150: GUI Engine Migration** — Refactor `simulation.c` and `root_gtk.c` to step and render from `physics_world` instead of global arrays.
- [ ] **151: Driver Station GUI** — Match timer, mode selection, telemetry readout, and field minimap.
- [ ] **152: Windows Platform Support (PLAT-003)** — GTK3/OpenGL compilation and packaging for Windows (required for FTC teams).
- [ ] **153: SIMD Math (PERF-001)** — Vectorize `math3D.h` with SSE/AVX intrinsics for field-scale performance.
- [ ] **154: Replay & Telemetry Export** — Record match physics states, replay, and export to CSV for autonomous tuning.

---

## Execution Rules

1. **Physics Before Product:** No FTC API or GUI work begins until Milestone 2 (Cylinder Wheels) and Milestone 3 (Solver) are physically validated.
2. **Test-Driven Fixes:** Every script must include a postflight verification (compile check or headless test assertion).
3. **No Blind Seds:** Complex logic changes are written as whole-file overwrites or targeted awk scripts, never blind regex replacements on logic blocks.
4. **Preserve the Core:** The motor model, battery sag, and `physics_world` context are load-bearing. Do not refactor them without explicit cause.
