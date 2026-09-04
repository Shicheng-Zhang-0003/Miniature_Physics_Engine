#!/usr/bin/env bash
# ============================================================
# FIX 058 — SAVE-001/SAVE-002: persist object IDs, remap joints on load
#   Scene format v150 writes each body's object_id. On load, an
#   old->new ID remap is built and joint references are rewritten
#   through it. Without this, saved joints reference IDs that no
#   longer exist and are silently deleted on the first tick.
# Phase:   phase0_foundation
# Files:   v15R2/src/config/mpe_constants.h
#          v15R2/src/scene/scene_saving.c
#          v15R2/src/scene/scene_load.c
# Depends: 054
# Risk:    medium (format change; 9 targeted edits, unique anchors,
#          requires the compile gate)
# ============================================================
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
CONST="v15R2/src/config/mpe_constants.h"
SAVE="v15R2/src/scene/scene_saving.c"
LOAD="v15R2/src/scene/scene_load.c"
REMAP_H="v15R2/src/scene/scene_id_remap.h"
for f in "$CONST" "$SAVE" "$LOAD" "$REMAP_H"; do
  [[ -f "$f" ]] || { echo "[SKIP] $f not found (is 054 applied?)"; exit 0; }
done
grep -q 'MPE_FTC_058' "$SAVE" && { echo "[SKIP] 058 already applied"; exit 0; }
grep -q '#define mpe_version               140' "$CONST" || { echo "[SKIP] mpe_version is not 140"; exit 0; }
grep -q 'write_int (f, rb -> static_state ? 1 : 0);' "$SAVE" || { echo "[SKIP] save anchor not found"; exit 0; }
grep -q 'if (!read_int (f, &static_int)) break;' "$LOAD" || { echo "[SKIP] load anchor not found"; exit 0; }

cp "$CONST" "${CONST}.pre_058"
cp "$SAVE"  "${SAVE}.pre_058"
cp "$LOAD"  "${LOAD}.pre_058"

# 1. Bump format version
sed -i 's/#define mpe_version               140/#define mpe_version               150/' "$CONST"

# 2. Save: persist object_id after static_state
sed -i 's|write_int (f, rb -> static_state ? 1 : 0);|write_int (f, rb -> static_state ? 1 : 0);\n        write_int (f, (int32_t) rb -> object_id); /* MPE_FTC_058 */|' "$SAVE"

# 3. Load: include the remap module
sed -i 's|#include "scene_init.h"|#include "scene_init.h"\n#include "scene_id_remap.h" /* MPE_FTC_058 */|' "$LOAD"

# 4. Load: keep accepting legacy v140 files
sed -i 's|(version != mpe_version \&\& version != 130)|(version != mpe_version \&\& version != 130 \&\& version != 140)|' "$LOAD"

# 5. Load: reset remap when the scene is cleared
sed -i 's|    scene_clear ();|    scene_clear ();\n    scene_id_remap_reset (); /* MPE_FTC_058 */|' "$LOAD"

# 6. Load: declare the saved-id scratch variable
sed -i 's|int32_t type_int, static_int;|int32_t type_int, static_int, saved_object_id = 0; /* MPE_FTC_058 */|' "$LOAD"

# 7. Load: read the persisted id (v150+ only)
sed -i 's|if (!read_int (f, \&static_int)) break;|if (!read_int (f, \&static_int)) break;\n        saved_object_id = 0;\n        if (version >= 150) { if (!read_int (f, \&saved_object_id)) {break;} } /* MPE_FTC_058 */|' "$LOAD"

# 8. Load: record old->new mapping after the fresh ID is assigned
sed -i 's|obj_per_scene \[i\].object_id = scene_allocate_object_id ();|obj_per_scene [i].object_id = scene_allocate_object_id ();\n        if ((version >= 150) \&\& (saved_object_id > 0)) { scene_id_remap_add ((uint32_t) saved_object_id, obj_per_scene [i].object_id); } /* MPE_FTC_058 */|' "$LOAD"

# 9. Load: resolve joint references through the remap
sed -i 's|add_joint_by_ids ((uint32_t) id_a, (uint32_t) id_b, eq, k, c);|add_joint_by_ids (scene_id_remap_resolve ((uint32_t) id_a), scene_id_remap_resolve ((uint32_t) id_b), eq, k, c); /* MPE_FTC_058 */|' "$LOAD"

# Postflight
grep -q '#define mpe_version               150' "$CONST" || { echo "[FAIL] version not bumped"; exit 1; }
grep -q 'MPE_FTC_058' "$SAVE" || { echo "[FAIL] save write not added"; exit 1; }
MARKS=$(grep -c 'MPE_FTC_058' "$LOAD")
[[ "$MARKS" -ge 6 ]] || { echo "[FAIL] expected >=6 markers in scene_load.c, found $MARKS"; exit 1; }
grep -q 'version != 140' "$LOAD" || { echo "[FAIL] legacy v140 acceptance missing"; exit 1; }
echo "[PASS] 058: scene format v150 — object IDs persisted, joints remapped on load"
