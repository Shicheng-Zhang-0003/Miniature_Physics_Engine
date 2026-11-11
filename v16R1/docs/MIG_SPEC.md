# MIG object spec — grammar v1 (FROZEN 2026-11-10, WP4 step 4.1)

This document IS the spec grammar. It is the frozen expansion of the
diagram in `MIG_BUILD_PLAN.md` §6 step 4.1; the diagram becomes this
reader's spec. Every consumer implements THIS text: the headless parser
(4.2), the microvim editing surface (4.3), and the GTK panel (4.4). The
grammar changes after 4.1 only by a dated v2 amendment — never by silent
extension, because a file written under v1 must parse identically
forever.

Principles carried over from the plan and charter (`NON_UNIFORM_BODY.md`
§6.1): the spec text is the single source of truth; no editor-only truth
exists; the reader rejects anything it does not recognise LOUDLY (a
named diagnostic), never guesses.

---

## 1. File rules

- UTF-8 text, one statement per line. Lines are split on whitespace
  (spaces and tabs); indentation is a human convention only — it carries
  NO syntax. Structure comes from keywords and the state machine in §3.
- `#` starts a comment; everything from `#` to end of line is ignored.
  A `#` inside a comment is fine; there are no string literals in v1,
  so `#` always begins a comment.
- Blank lines and comment-only lines are ignored anywhere.
- Numbers: C locale, decimal, optional sign, optional exponent
  (`0.05`, `-1.25`, `2.2e3`). No hex, no `inf`, no `nan`, no thousands
  separators, no locale comma. Anything else is `mig_spec_bad_number`.
- Case-sensitive keywords: `Part` is an unknown keyword.
- Maximum line length 512 bytes; a longer line is `mig_spec_trailing`
  (it cannot be a v1 statement).

## 2. Statements

Exactly five statement shapes (fields below attach to the current
scope):

```
mig <name>
frame assembly
part <index> type sphere|cube|cylinder
<part-field> ...        # dims / pose / material / mass|density / colour
```

`<name>` is 1..63 bytes of `[A-Za-z0-9_-]`; anything else is
`mig_spec_bad_name`. The name is the key of the per-session spec store
(4.2) — it is NOT the spawned body's scene id.

## 3. Structure (the reader's state machine)

1. The FIRST statement must be `mig <name>` — it opens the object.
   Before it, only comments/blank lines are allowed. A part-field or
   `frame`/`part` line before `mig` is `mig_spec_no_object`.
2. `mig` may appear exactly once (`mig_spec_dup_object`). A second
   `mig` line does not start a second object: one file, one object.
3. `frame <value>`: optional, at most once (`mig_spec_dup_kw`), must
   appear before the first `part`. v1 accepts EXACTLY ONE value:
   `assembly`. Any other value (`com`, `world`, `local`, …) is
   `mig_spec_bad_frame` — the reader never guesses a frame. `frame
   assembly` declares: the part poses below are in the ASSEMBLY frame
   (the caller's definition frame), and the COM is computed when the
   spec becomes a body — the body is always born COM-framed (charter
   decision (A); the re-anchor is `mig_rebase_parts` on the caller
   side, not a physics step). The default when the line is absent is
   `assembly` (it is the only frame v1 has; the line exists so files
   say what they mean).
4. `part <index> type <t>` opens a part scope and switches the current
   part. Part-scoped fields (`dims`, `pose`, `material`, `mass`,
   `density`, `colour`) attach to the MOST RECENT `part` line.
5. Part indices must be exactly `0, 1, 2, …, n-1`, strictly increasing,
   each exactly once. The index is definitional order — it is the
   per-body monotonic index the composite pool stores (charter §2:
   indices are never reused). Out-of-order, duplicate, or gapped
   indices are `mig_spec_bad_part_index`; a file that means `0,2`
   must say `0,1,2` (the reader does not renumber).
6. Exactly one or more parts; at least one part is required
   (`mig_spec_no_parts`) and the hard ceiling is
   `composite.max_parts` (the P2 cap, 16). Over the cap is
   `mig_spec_part_cap` — the spec is a DEFINITION; the counted-clamp
   pool behaviour (keep first max_parts) is a runtime property of the
   pool, not of a file, so a file over the cap is rejected, not
   truncated.
7. A part-field with no current part is `mig_spec_field_outside_part`.
8. Nothing may follow the last part except more part scopes, their
   fields, comments, and blank lines. There are no object-scoped
   trailing statements in v1; an unrecognised keyword anywhere is
   `mig_spec_unknown_kw` with the offending word quoted.

## 4. Part fields (per part)

### `dims` — REQUIRED, exactly one per part (`mig_spec_dup_kw` / `mig_spec_missing_dims`)

The subset is fixed PER TYPE; a key from another type's subset is
`mig_spec_dims_wrong_type` (never ignored):

| type     | keys                                  | engine field |
|----------|---------------------------------------|--------------|
| sphere   | `radius r`                            | `radius` |
| cube     | `hx a hy b hz c` (all three required) | `half_extensions` |
| cylinder | `radius r` + `half hl` (both required)| `radius`, `cylinder_half_length` |

`half` is the cylinder HALF-LENGTH along the part's local X axle (the
engine's MPE_FTC_090 convention). All dims must be finite and > 0
(`mig_spec_bad_dims` — the same rule `mig_compose_bad_dims` enforces).

### `pose` — REQUIRED, exactly one per part

```
pose px py pz qw qx qy qz
```

Position in the assembly frame; orientation is the part→assembly
quaternion in the engine (w, x, y, z) convention — w FIRST. The
quaternion must be unit within 1±1e-4 (the `mig_compose_bad_quat`
threshold): the reader ASSERTS unit-ness and rejects, it NEVER
renormalises (renormalising would hide an authoring bug behind a silent
substitution). Non-finite components are `mig_spec_bad_pose`; a
non-unit quaternion is `mig_spec_bad_quat`.

### `material` — optional, at most one per part

```
material mu_s F mu_k F rest F
```

All three keys required on the one line. Values finite, `mu_s ≥ 0`,
`mu_k ≥ 0`, `rest ∈ [0,1]` (`mig_spec_bad_material`). Default when the
line is absent: the `body_defaults` friction/restitution of the part's
own primitive type — exactly what a primitive spawn of that type gets.
(`mu_k > mu_s` is not enforced; the solver takes the pair as authored.)

### `mass` XOR `density` — REQUIRED, exactly one per part

```
mass kg        # explicit kilograms
density rho    # kg/m^3; volume from the exact primitive dims
```

Exactly one of the two keywords per part; both present is
`mig_spec_mass_xor_density`, neither is `mig_spec_missing_mass`. Value
finite and > 0 (`mig_spec_bad_mass` — matches `mig_compose_bad_mass`).
Density mode uses the SAME exact volume formulas as `mig_part_mass`
(sphere 4/3·πr³, cube 8·hx·hy·hz, cylinder πr²·(2·half)); there is no
second volume math anywhere in the pipeline.

### `colour` — optional, at most one per part

```
colour r g b
```

Floats in [0,1] (`mig_spec_bad_colour` outside). Default when absent:
(1, 1, 1). This field is the WP4 arrival of per-part authored colour —
before WP4, composites drew the parent body's colour on every part
(plan §6 step 2.7 note). The body-level colour remains the envelope's
(diagnostics/gizmos); the part colour is what the instanced draw packs.

## 5. Frame rule and the round-trip contract (what 4.2's gate pins)

- The FILE stores assembly-frame poses. When the spec becomes a body,
  `mig_make_bridge` composes and re-anchors a COPY to the COM frame;
  the definition frame in the file is untouched by that act.
- SAVING a body back to spec re-emits assembly-frame poses (the
  un-rebase: part position + body COM) and the composed totals are NOT
  written — mass is re-derived from the file's `mass`/`density` lines
  on every load (the WP3 loader-kindness rule: the reader never trusts
  derived truth, it recomputes through the compose path).
- NORMALIZED OUTPUT is canonical: statements in the order `mig`,
  `frame` (always emitted), then per part in index order: `part`,
  `dims`, `pose`, `material` (only if authored), `mass`|`density`,
  `colour` (only if authored); single spaces; numbers in shortest
  round-trip form. Read + write must reproduce byte-identical
  normalized output — that is gate 4.2's acceptance bar, and it is why
  §1–§4 above are pinned to the byte, not the spirit.

## 6. Named diagnostics (source `mig_spec`, mpe_diag codes)

The reader reports through the engine diag discipline (source, stable
machine code ≤ 24 chars, call site, human message with the offending
line number and token). Codes frozen at v1:

| code | fires when |
|------|------------|
| `mig_spec_empty_file` | no statements at all |
| `mig_spec_no_object` | statement before `mig` |
| `mig_spec_dup_object` | second `mig` line |
| `mig_spec_bad_name` | name outside `[A-Za-z0-9_-]` or > 63 bytes |
| `mig_spec_dup_kw` | repeated `frame`/`dims`/`pose`/`material`/`colour` in one scope |
| `mig_spec_bad_frame` | `frame` value ≠ `assembly` |
| `mig_spec_bad_part_index` | index out of order, duplicated, gapped, or negative |
| `mig_spec_part_cap` | more parts than `composite.max_parts` |
| `mig_spec_no_parts` | zero `part` lines |
| `mig_spec_field_outside_part` | part-field with no current part |
| `mig_spec_unknown_kw` | any unrecognised leading keyword |
| `mig_spec_bad_type` | `type` ≠ sphere/cube/cylinder |
| `mig_spec_missing_type` | `part` line without `type` |
| `mig_spec_dims_wrong_type` | dims key from another type's subset |
| `mig_spec_missing_dims` | part with no `dims` line |
| `mig_spec_bad_dims` | non-finite / non-positive dim |
| `mig_spec_missing_pose` | part with no `pose` line |
| `mig_spec_bad_pose` | non-finite pose component |
| `mig_spec_bad_quat` | orientation outside 1±1e-4 |
| `mig_spec_bad_material` | missing key, non-finite, or rest ∉ [0,1] |
| `mig_spec_mass_xor_density` | both `mass` and `density` on one part |
| `mig_spec_missing_mass` | neither `mass` nor `density` |
| `mig_spec_bad_mass` | mass/density non-finite or ≤ 0 |
| `mig_spec_bad_colour` | colour ∉ [0,1] or wrong count |
| `mig_spec_bad_number` | unparsable numeric token |
| `mig_spec_trailing` | line > 512 bytes / trailing tokens on a statement |

A file with ANY reject is not half-loaded: nothing enters the spec
store from a rejected file (write-on-success, the `mig_compose` rule).
Compose-level failures that survive parsing (they cannot, for
well-formed dims/mass — but the defence is layered) surface verbatim as
the `mig_compose_*` codes at commit time.

## 7. Frozen OUT of v1 (named, so absence is a decision not an omission)

Joints between objects; per-part `no_collide` (it stays body-level,
plan 2.7); velocity/sleep/static/kinematic authoring (WP3 scene
persistence owns dynamic state; the spec owns SHAPE); convex hulls and
mesh import (charter §8, next train); principal-axis display (plan
4.4's named TODO); units other than metres/kilograms/seconds; multiple
objects per file (one file, one object — share by copying files, not by
growing a container format).
