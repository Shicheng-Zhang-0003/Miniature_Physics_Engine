#ifndef mig_spec_h
#define mig_spec_h
/* MIG WP4 step 4.2 — the frozen grammar reader (docs/MIG_SPEC.md).
 *
 * Pure parser: knows nothing of physics_world, rigidbody, the config
 * store or mig_mass. It reads a MIG object spec file and fills a
 * caller-owned mig_spec; on any reject NOTHING is written to out
 * (write-on-success, the mig_compose rule). The caller then decides
 * what to do with the spec: store it (terminal verbs), emit normalized
 * text (the byte-identical round-trip target), or convert it to
 * mig_parts and compose it into a body (WP4 later steps).
 *
 * Determinism: single-threaded parse, fixed token order, no malloc
 * inside parse hot paths (fixed caller-supplied buffers), structural
 * and numeric validation only.
 */
#include <stddef.h>

/* Hard ceiling for the in-spec part list — the P2 cap, mirrored here so
 * the parser can enforce "a file over the cap is REJECTED, not truncated"
 * without reaching into the config subsystem. The engine's real cap is
 * composite.max_parts; callers pass it in (default 16). */
#define MIG_SPEC_MAX_PARTS 16
#define MIG_SPEC_NAME_LEN 64
#define MIG_SPEC_LINE_CAP 512

/* One part's authored definition, exactly as the grammar allows. All
 * numeric fields are doubles internally (the float boundary is the
 * engine struct, not the spec file). */
typedef struct {
    int type;            /* mig_part_sphere/cube/cylinder values */
    /* dims (per-type subset of the grammar) */
    double radius;
    double cylinder_half_length;
    double half_extensions_x, half_extensions_y, half_extensions_z;
    /* pose in the assembly frame, w-first */
    double position_x, position_y, position_z;
    double orientation_w, orientation_x, orientation_y, orientation_z;
    /* mass source */
    int mass_mode;       /* mig_mass_mode_explicit / mig_mass_mode_density */
    double mass;         /* explicit kg, or density kg/m^3 */
    int has_mass;        /* 1 once a `mass` or `density` line was seen */
    /* optional fields — presence flags */
    int has_material;
    int has_dims;
    int has_pose;
    int has_colour;
    double mu_s, mu_k, restitution;
    double colour_r, colour_g, colour_b;
} mig_spec_part;

typedef struct {
    char name [MIG_SPEC_NAME_LEN];
    int has_frame;       /* 1 if a `frame assembly` line was present */
    mig_spec_part parts [MIG_SPEC_MAX_PARTS];
    int count;
} mig_spec;

/* Named reject codes, source "mig_spec", mpe_diag discipline
 * (source + stable code <= 24 chars + call site + message with the
 * offending line number and token). */
typedef enum {
    mig_spec_ok = 0,
    mig_spec_empty_file = 1,     /* no statements at all */
    mig_spec_no_object = 2,      /* statement before `mig` */
    mig_spec_dup_object = 3,     /* second `mig` line */
    mig_spec_bad_name = 4,       /* name outside [A-Za-z0-9_-] or > 63 bytes */
    mig_spec_dup_kw = 5,         /* repeated frame/dims/pose/material/colour in one scope */
    mig_spec_bad_frame = 6,      /* `frame` value != `assembly` */
    mig_spec_bad_part_index = 7, /* index out of order, duplicated, gapped, negative */
    mig_spec_part_cap = 8,       /* more parts than max_parts */
    mig_spec_no_parts = 9,       /* zero `part` lines */
    mig_spec_field_outside_part = 10, /* part-field with no current part */
    mig_spec_unknown_kw = 11,    /* any unrecognised leading keyword */
    mig_spec_bad_type = 12,      /* `type` != sphere/cube/cylinder */
    mig_spec_missing_type = 13,  /* `part` line without `type` */
    mig_spec_dims_wrong_type = 14, /* dims key from another type's subset */
    mig_spec_missing_dims = 15,  /* part with no `dims` line */
    mig_spec_bad_dims = 16,      /* non-finite / non-positive dim */
    mig_spec_missing_pose = 17,  /* part with no `pose` line */
    mig_spec_bad_pose = 18,      /* non-finite pose component */
    mig_spec_bad_quat = 19,      /* orientation outside 1±1e-4 */
    mig_spec_bad_material = 20,  /* missing key, non-finite, or rest not in [0,1] */
    mig_spec_mass_xor_density = 21, /* both `mass` and `density` on one part */
    mig_spec_missing_mass = 22,  /* neither `mass` nor `density` */
    mig_spec_bad_mass = 23,      /* mass/density non-finite or <= 0 */
    mig_spec_bad_colour = 24,    /* colour not in [0,1] or wrong count */
    mig_spec_bad_number = 25,    /* unparsable numeric token */
    mig_spec_trailing = 26       /* line > 512 bytes / trailing tokens on a statement */
} mig_spec_err;

/* Parse file `path` into `out`, enforcing the part ceiling `max_parts`
 * (engine default 16). Returns mig_spec_ok on success; on any other
 * return value `out` is untouched (write-on-success). Line numbers and
 * offending tokens are reported through mpe_diag (source "mig_spec"). */
int mig_spec_parse_file (const char *path, int max_parts, mig_spec *out);

/* Emit the canonical normalized form of `spec` into `buf` (cap bytes,
 * NUL-terminated when cap > 0). Returns bytes written excluding NUL;
 * if cap is too small, returns the required size without writing.
 * Normalized order: mig, frame (always emitted), then per part in index
 * order: part, dims, pose, material (only if authored), mass|density,
 * colour (only if authored); single spaces; shortest round-trip numbers.
 * Read + write must reproduce byte-identical output — gate 4.2 pins it. */
size_t mig_spec_emit_normalized (const mig_spec *spec, char *buf, size_t cap);

#endif /* mig_spec_h */
