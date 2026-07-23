#!/usr/bin/env python3
from pathlib import Path
import sys

SIM = Path("src/simulation.c")

if not SIM.is_file():
    sys.exit(f"ERROR: expected {SIM} to exist. Run this from the folder containing src/.")

text = SIM.read_text(encoding="utf-8")
original = text

new_function = (
    "static void validation_report_print (void) {\n"
    "    /* A3_PATCH_51_FIX_VALIDATION_PRINT */\n"
    "    printf (\"[A3] Validation report %s\\n\", A3_VERSION_STRING);\n"
    "    printf (\"[A3] objects=%d capacity=%d joints=%d selected=%d\\n\",\n"
    "            object_count, object_capacity, current_joint_count, selected_object);\n"
    "    printf (\"[A3] debug last: obj=%d pairs=%d manifolds=%d frame_time=%f\\n\",\n"
    "            debug_last_object_count,\n"
    "            debug_last_broadphase_pair_count,\n"
    "            debug_last_manifold_count,\n"
    "            debug_last_frame_time);\n"
    "    printf (\"[A3] broadphase overflow: nodes=%d pairs=%d\\n\",\n"
    "            broadphase_get_node_overflow_count (),\n"
    "            broadphase_get_pair_overflow_count ());\n"
    "    printf (\"[A3] contact cache: hits=%d misses=%d\\n\",\n"
    "            contact_cache_get_hits (),\n"
    "            contact_cache_get_misses ());\n"
    "    printf (\"[A3] menus: open=%d spawner=%d velocity=%d object=%d marked_joint=%d\\n\",\n"
    "            main_inputs.is_menu_open,\n"
    "            main_inputs.spawner_menu_level,\n"
    "            main_inputs.velocity_menu_level,\n"
    "            main_inputs.object_menu_level,\n"
    "            main_inputs.marked_joint_object_index);\n"
    "    fflush (stdout);\n"
    "}\n"
)

start_marker = "static void validation_report_print (void) {"
start_index = text.find(start_marker)

if start_index != -1:
    brace_index = text.find("{", start_index)

    if brace_index == -1:
        sys.exit("ERROR: could not find opening brace for validation_report_print().")

    depth = 0
    end_index = -1

    for i in range(brace_index, len(text)):
        character = text[i]

        if character == "{":
            depth += 1
        elif character == "}":
            depth -= 1

            if depth == 0:
                end_index = i + 1
                break

    if end_index == -1:
        sys.exit("ERROR: could not find closing brace for validation_report_print().")

    text = text[:start_index] + new_function + text[end_index:]

else:
    anchor = "gboolean physics_step_increment (gpointer user_data_pointer) {"
    anchor_index = text.find(anchor)

    if anchor_index == -1:
        sys.exit(
            "ERROR: could not find validation_report_print() or physics_step_increment() in src/simulation.c\n"
            "Run this to inspect it:\n"
            "    grep -n \"validation_report_print\" src/simulation.c"
        )

    text = text[:anchor_index] + new_function + "\n" + text[anchor_index:]

if text == original:
    print("No changes were necessary.")
    sys.exit(0)

backup = SIM.with_name(SIM.name + ".bak_a3_51_fix")
if not backup.exists():
    backup.write_text(original, encoding="utf-8")
    print(f"Backup written: {backup}")

SIM.write_text(text, encoding="utf-8")
print(f"Fixed validation_report_print in {SIM}")
