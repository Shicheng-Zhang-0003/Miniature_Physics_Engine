# MPE Multicore — v15S

> Threading model, determinism guarantees, and how to run the solver on
> multiple cores.

---

## 1. Current State

**Multicore phases are implemented** (2026-10-15). With `MPE_THREADS > 1`:

- **Island-based solver sweeps**: each active island is a work item in the
  persistent pool. Islands are dispatched in batches of at most eight; the
  Gauss-Seidel order within each island remains serial.
- **Per-body integration** parallelized via `mpe_parallel_integrate()`.
- **Per-body sanitize** and the two-phase CCD sweep are parallelized.
- **Narrowphase detection** writes independent pair results in parallel.
  Wake decisions, contact flags, and manifold preparation run serially in
  broadphase pair order because several pairs may share a body.
- **Joint solve** (`constraint_solve_all`) stays serial — O(1024) joint slot
  scan, measured negligible vs manifold solve; joints only connect bodies
  within one island, so no cross-island dependency exists.

**Result: bitwise identical to serial at tested thread counts.** Determinism
twins have been checked at 1 / 4 / 8 threads. Re-verified 2026-10-20 (same
post-atomic-shim pass): `test_paranoia_determinism` logs are md5-identical
across MPE_THREADS ∈ {1, 4, 8} on this container — twin 1200-tick worlds,
re-init, 10-run bitwise reference, concurrent-vs-serial, NaN gate all PASS
at each setting. The pool is process-wide; dispatch phases are serialized
across simultaneous `physics_world_step` calls, while each world's data
remains independent. Pair handlers can be called concurrently and must
follow the thread-safety contract in [`MODULES.md`](MODULES.md).

---

## 2. Why Islands Parallelize Cleanly

Solver islands are connected components of the contact+joint graph. By
construction, **no body belongs to two islands**. Therefore:

- Each island's manifolds are solved by exactly one thread.
- Writes are disjoint across threads (thread A writes bodies in island A;
  thread B writes bodies in island B; no overlap).
- Within-island Gauss-Seidel order is preserved (manifolds solved in sort
  order, 2 visits each).
- Thread scheduling never affects results: disjoint writes + fixed within-
  island order = bitwise identical to serial, on every thread count.

---

## 3. How It Works (`core/physics_world.c`)

### 3.1 Thread count

```
MPE_THREADS env var → strtol + clamp to [1, 8]
unparseable → 1 (serial)
threads ≤ 1 or body_count < 64 → serial fast path
```

`atoi()` was replaced with `strtol()` + clamp (DESPOT-2026-10-13: `atoi()`
is UB-adjacent on overflow garbage and gives no error signal).

### 3.2 Parallel solve sweep

For each solver iteration:

1. Assign each manifold to its island (via body A).
2. Count manifolds per island; build prefix-sum offset table.
3. Scatter manifold indices into per-island buckets, preserving sort order.
4. Dispatch one pool task per awake island that has manifolds (up to 8 tasks
   per batch, matching `pthread_t tids[8]`).
5. Each thread runs `mpe_solve_island_manifolds()`: solves its island's
   manifolds in sort order, 2 visits each, using `mpe_step_resolve()`.
6. Join all threads before the serial joint solve (`constraint_solve_all`),
   so the next phase sees consistent state.

Fallback: if `pthread_create` fails for a slice, that island is solved
serially (never silently skipped).

### 3.3 Parallel integration

Same disjoint-write pattern: each thread integrates a contiguous slice of
bodies. Each body's new state depends only on its own pre-integration state
(position, velocity, CCD remainder, joint membership, has_contact flag), so
order within the integration phase is irrelevant — bitwise identical to
serial at any thread count.

### 3.4 What stays serial

| Phase | Reason |
|---|---|
| Broadphase pair generation | shared pair buffer write |
| Narrowphase wake/contact bookkeeping and manifold preparation | shared body state; detection itself writes disjoint per-pair slots in parallel |
| Joint solve (`constraint_solve_all`) | O(1024) scan, measured negligible |
| Poisson / split / rolling | shared manifold and joint state; not yet partitioned by island |
| Depenetration / boundary | shared body and contact state; not yet partitioned by body |

---

## 4. Determinism

- GCC/Clang: bitwise twins at any thread count (proven).
- MSVC: needs `/fp:strict` (no FMA contraction) for the same guarantee —
  **unproven on Windows**. Scope bitwise-twin claims to GCC/Clang until
  MSVC flags land in CI.
- `ENABLE_NATIVE=0` required for bitwise twins; `ENABLE_NATIVE=1`
  (`-march=native`) breaks bitwise portability.
- `-ffp-contract=off -fno-fast-math` hard-appended via `override CFLAGS +=`
  so environment or command-line CFLAGS can never silently drop them.

---

## 5. Running

```bash
# Serial (default)
MPE_THREADS=1 ./engine

# 4 threads
MPE_THREADS=4 ./engine

# 8 threads
MPE_THREADS=8 ./engine
```

Determinism test (headless, no GTK):

```bash
cd v15S/src
make test_determinism
MPE_THREADS=1 ./test_determinism   # PASS: 600 ticks bitwise identical
MPE_THREADS=4 ./test_determinism   # PASS
MPE_THREADS=8 ./test_determinism   # PASS
```

---

## 6. Future Work

- **Within-island parallel solve**: preserve the current serial Gauss-Seidel
  order, or use graph coloring/dependency tracking before parallelizing rows
  inside a connected island.
- **TSan**: the full GCC/Linux suite and concurrent-world regression pass with
  `setarch x86_64 -R`, which avoids this container's TSan shadow-memory startup
  collision. MSVC and non-Linux toolchains remain unverified.
- **SIMD SoA batching**: true 4-wide struct-of-arrays batching is the
  remaining speedup frontier, not per-op SIMD wrapping (lane ops are exactly
  rounded; reductions stay scalar by proof).
