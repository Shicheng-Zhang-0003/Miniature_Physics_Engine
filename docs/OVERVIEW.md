# Overview

MPE is a custom-built **3D rigid-body physics engine and real-time rendering
pipeline**, written in **C**. The headless physics core uses libc, libm, libdl,
and pthreads. The interactive engine adds GTK4 and OpenGL through libepoxy;
GTK4 selects X11 or Wayland at runtime. `mpe-tui` additionally needs ncurses.

MPE is built around four priorities:

- **Mathematical transparency** — every integrator, solver, and collision test
  is hand-written and inspectable.
- **Cache-efficient data layouts** — tightly packed structs and contiguous
  instance buffers.
- **Deterministic simulation** — fixed-timestep physics decoupled from render
  framerate; bitwise-identical tested runs on supported GCC/Clang IEEE builds.
- **Real-time scaling** — GPU instancing and a spatial-hash broadphase with
  expected near-linear behavior for ordinary spatial distributions; dense
  cells and configured capacity limits can still increase work.

The module system can replace the broadphase and solver, register narrowphase
shape-pair handlers, and attach per-world tick hooks. Built-in joints remain
engine-managed, and force fields can be implemented through tick hooks; they
are not separate replaceable pipeline stages. See [the module guide](../v16R1/MODULES.md)
for the supported extension points and ABI rules.

The complete step order and data ownership are in
[the architecture guide](../v16R1/ARCHITECTURE.md). Exact model equations,
known approximations, and measured numerical budgets are in
[the numerical guarantees](../v16R1/docs/NUMERICAL_GUARANTEES.md). The audited
test matrix and current run artifacts are described in
[verification](VERIFICATION.md).
