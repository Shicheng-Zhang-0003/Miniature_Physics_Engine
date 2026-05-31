# Miniature Physics Engine - Release 1.1-Stable

## Summary

This release focuses on stability, menu usability, collision behaviour, scene loading safety, and internal code organisation.

## Menu Changes

- Pressing 7 opens the user/world mechanics menu.
- Pressing 7 again closes the user/world mechanics menu.
- Pressing 8 opens the object spawning menu.
- Pressing 8 again closes the object spawning menu.
- Pressing 9 opens the save/load/exit menu.
- Pressing 9 again closes the save/load/exit menu.
- After a numerical value is changed, the active menu closes automatically.
- Numerical menu dialogs no longer return to the previous submenu after closing.
- Selected object value edits now also close the object menu after completion.

## Physics Changes

- Collision impulse handling and penetration correction are now separated.
- Penetration correction can run even when two overlapping objects are moving apart.
- Static and dynamic body correction now uses inverse mass distribution more consistently.
- Broadphase collision pairs are regenerated inside each physics substep.
- Substep collision detection is more consistent for moving bodies.

## Scene Loading Changes

- Scene loading now checks the object count before applying the load.
- Scene loading now checks every expected value from the file.
- Invalid scene data no longer replaces the active scene halfway through loading.
- Invalid object types are rejected.
- Static object mass is preserved during load.
- Static objects still correctly receive zero inverse mass after loading.
- Loaded orientations are normalised before use.
- Loaded object axes are refreshed after loading.

## Code Organisation Changes

- Rigidbody implementation code moved from buffer.h into buffer.c.
- Force implementation code moved from define_forces.h into define_forces.c.
- Collision implementation code moved from collision_mechanics.h into collision_mechanics.c.
- Headers now focus on declarations, structures, and shared types.
- The makefile now compiles the new C source files.
- Compiler warning output is substantially cleaner.

## Small Cleanups

- Removed an unused simulation variable.
- Marked unused GTK callback parameters intentionally unused.
- Kept source formatting consistent with the existing project style.

## Verification

- Forced rebuild completed successfully with make -B.
- Diff whitespace check completed successfully.
