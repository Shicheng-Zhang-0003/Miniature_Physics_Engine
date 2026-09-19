/* Phase-4: `mod` terminal command — hot plug-and-play physics modules.
 * Usage:
 *   mod ls                        list registry modules + loaded .so
 *   mod load <path.so>            dlopen plugin (registers shape/solver/etc)
 *   mod unload <path|name>        dlclose (quiesce between ticks)
 *   mod attach <name>             attach tick module to primary world
 *   mod detach <name>             detach tick module
 *   mod use-broadphase <name|builtin>  swap broadphase backend
 *   mod use-solver <name|builtin>      swap solver backend
 */
#include "term_priv.h"
#include "../core/mpe_registry.h"
#include "../core/mpe_loader.h"
#include "../core/physics_world.h"
#include <stdio.h>
#include <string.h>

void cmd_mod(int argc, char **argv) {
    if (argc < 2) {
        term_err("mpe: mod: usage: mod ls|load|unload|attach|detach|use-broadphase|use-solver\n");
        return;
    }
    if (term_str_eq(argv[1], "ls")) {
        char buf[256];
        term_out("registry modules:\n");
        for (int i = 0; i < mpe_module_count(); i++) {
            const mpe_module_desc_t *d = mpe_module_at(i);
            if (!d) continue;
            snprintf(buf, sizeof(buf), "  %s-%s [%s]%s\n", d->name, d->version ? d->version : "?",
                     d->kind ? d->kind : "generic", d->deterministic ? " det" : "");
            term_out(buf);
        }
        snprintf(buf, sizeof(buf), "loaded .so: %d\n", mpe_loader_count());
        term_out(buf);
        for (int i = 0; i < mpe_loader_count(); i++) {
            snprintf(buf, sizeof(buf), "  %s\n", mpe_loader_path_at(i));
            term_out(buf);
        }
        physics_world *w = physics_world_get_primary();
        snprintf(buf, sizeof(buf), "attached to primary: %d\n", w ? w->tick_module_count : 0);
        term_out(buf);
        if (w) for (int i = 0; i < w->tick_module_count; i++) {
            snprintf(buf, sizeof(buf), "  %s\n", w->tick_modules[i] ? w->tick_modules[i]->name : "?");
            term_out(buf);
        }
        return;
    }
    if (term_str_eq(argv[1], "load") && argc >= 3) {
        char err[512] = {0};
        if (mpe_loader_load(argv[2], err, sizeof(err)) == 0) term_ok("mpe: mod: loaded\n");
        else { char b[600]; snprintf(b, sizeof(b), "mpe: mod: load failed: %s\n", err); term_err(b); }
        return;
    }
    if (term_str_eq(argv[1], "unload") && argc >= 3) {
        if (mpe_loader_unload(argv[2]) == 0) term_ok("mpe: mod: unloaded\n");
        else term_err("mpe: mod: unload failed (unknown handle)\n");
        return;
    }
    if (term_str_eq(argv[1], "attach") && argc >= 3) {
        const mpe_module_desc_t *d = mpe_find_module(argv[2]);
        if (!d) { term_err("mpe: mod: unknown module\n"); return; }
        int r = physics_world_attach_module(physics_world_get_primary(), d);
        if (r >= 0) term_ok("mpe: mod: attached\n");
        else term_err("mpe: mod: attach failed (table full / attach hook)\n");
        return;
    }
    if (term_str_eq(argv[1], "detach") && argc >= 3) {
        if (physics_world_detach_module(physics_world_get_primary(), argv[2]) == 0) term_ok("mpe: mod: detached\n");
        else term_err("mpe: mod: detach failed (not attached)\n");
        return;
    }
    if (term_str_eq(argv[1], "use-broadphase") && argc >= 3) {
        if (term_str_eq(argv[2], "builtin")) {
            physics_world_set_broadphase(physics_world_get_primary(), NULL);
            term_ok("mpe: mod: broadphase=builtin\n"); return;
        }
        const mpe_broadphase_if_t *b = mpe_find_broadphase(argv[2]);
        if (!b) { term_err("mpe: mod: unknown broadphase\n"); return; }
        physics_world_set_broadphase(physics_world_get_primary(), b);
        term_ok("mpe: mod: broadphase swapped\n"); return;
    }
    if (term_str_eq(argv[1], "use-solver") && argc >= 3) {
        if (term_str_eq(argv[2], "builtin")) {
            physics_world_set_solver(physics_world_get_primary(), NULL);
            term_ok("mpe: mod: solver=builtin\n"); return;
        }
        const mpe_solver_if_t *s = mpe_find_solver(argv[2]);
        if (!s) { term_err("mpe: mod: unknown solver\n"); return; }
        physics_world_set_solver(physics_world_get_primary(), s);
        term_ok("mpe: mod: solver swapped\n"); return;
    }
    term_err("mpe: mod: unknown subcommand\n");
}
