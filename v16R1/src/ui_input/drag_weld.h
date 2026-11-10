#ifndef drag_weld_h
#define drag_weld_h
#include <stdbool.h>
/* SANDBOX S4b: grab-and-carry. LMB (while the mouse is locked) selects
 * whatever the crosshair aims at and GRABS it at its current distance
 * along the camera forward ray; while held, the body rides that ray
 * (position follows the camera every frame, velocities zeroed, body
 * woken); release drops it. S4c turns the release into a weld when the
 * carried body overlaps another. Indices shift when bodies are removed,
 * so the drag is keyed to the stable object id, re-resolved per tick. */
void drag_weld_press (void);  /* consume the LMB press edge: select + grab */
void drag_weld_tick (void);   /* every frame while held: follow the camera */
void drag_weld_release (void);/* consume the LMB release edge: drop (S4c: weld) */
bool drag_weld_is_active (void);
#endif /* drag_weld_h */
