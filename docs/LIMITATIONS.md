# Known Limitations

See [`v16R1/KNOWN_LIMITATIONS.md`](../v16R1/KNOWN_LIMITATIONS.md) for the
complete, honest list. Headlines:

- **Warm start:** cold by default (stable across all regimes). Damped
  warm-start is experimental — faster convergence at low iteration counts
  but unstable under extreme gravity/friction/restitution.
- **CCD:** translation-only for volume-object pairs; floor plane includes
  rotational lowest-point velocity. Rotation-only tunneling between
  arbitrary volumes is unsolved.
- **Multithreading:** per-body work, CCD, pair-local collision detection, and
  solver sweeps across independent islands use `MPE_THREADS`; shared-body
  contact preparation and joint solving remain serial. MSVC `/fp:strict`
  determinism is unproven.
- **GUI vs headless step paths:** separate frame drivers (accumulator vs
  direct dt). Bitwise identical on passive towers; driven robots differ
  ~2.7% via motor-observer feedback (expected, not a path bug).
- **Jointed air-spin limit cycle:** free-spinning jointed wheels contain a
  residual limit cycle, contained by bearing damping (c=1e-6) + diode+slew.
  Documented frontier.
- **S2 bridge:** staged bodies are kinematic display objects and refreshes
  move them in place; bonds remain k=0. Physical coupling (positions flowing
  back to S2) remains out of scope.
