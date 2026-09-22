# Remaining Work

## High priority

- [ ] Repair the remaining paranoia tests and make their mathematical oracles valid.
- [ ] Diagnose and fix cylinder collision and sleep/depenetration failures with minimal reproducible cases.
- [ ] Add a regression test for plugin load/attach/unload safety and stage-backend lifetime handling.
- [ ] Replace the duplicate GUI and headless physics pipelines with one canonical step path.

## Correctness and validation

- [ ] Add valid CCD, constraint, friction, restitution, and free-flight invariant tests with documented tolerances.
- [ ] Build and run every paranoia target, including energy/momentum, scene persistence, and spring-joint tests.
- [ ] Run the complete suite under AddressSanitizer and UndefinedBehaviorSanitizer.
- [ ] Add randomized/property-based physics tests and differential checks for simple analytic cases.

## Operational and release hygiene

- [ ] Fix stale Makefile/help targets, including the advertised module target.
- [ ] Define and test module stage detachment before unloading shared libraries.
- [ ] Document numerical guarantees and unsupported CCD/rotational cases precisely.
- [ ] Add a root README and update release gates to reflect verified behavior.
- [ ] Decide and document thread-safety boundaries for global registries and configuration.
