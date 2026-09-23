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
            const char *nm = mpe_loader_name_at(i);
            snprintf(buf, sizeof(buf), "  %s (%s)\n", mpe_loader_path_at(i), nm ? nm : "?");
            term_out(buf);
        }
        /* Pair-handler table (foreign shapes visible here). */
        term_out("pair handlers:\n");
        for (int i = 0; i < MPE_MAX_PAIR_HANDLERS; i++) {
            int ta = 0, tb = 0, ca = 0, cb = 0;
            char nm[64] = {0};
            if (mpe_registry_pair_describe(i, &ta, &tb, &ca, &cb, nm, sizeof(nm)) != 0) break;
            snprintf(buf, sizeof(buf), "  #%d (%d,%d,%d,%d) %s\n", i, ta, tb, ca, cb, nm);
            term_out(buf);
        }
        /* Stage backends. */
        if (mpe_find_broadphase("hash")) term_out("broadphase: hash (builtin)\n");
        if (mpe_find_solver("seq-impulse")) term_out("solver: seq-impulse (builtin)\n");
        physics_world *w = physics_world_get_primary();
        snprintf(buf, sizeof(buf), "attached to primary: %d\n", w ? w->tick_module_count : 0);
        term_out(buf);
        if (w) for (int i = 0; i < w->tick_module_count; i++) {
            snprintf(buf, sizeof(buf), "  %s\n", w->tick_modules[i] ? w->tick_modules[i]->name : "?");
            term_out(buf);
        }
        if (w) {
            snprintf(buf, sizeof(buf), "primary stages: broadphase=%s solver=%s\n",
                     w->broadphase_if ? "foreign" : "builtin",
                     w->solver_if ? "foreign" : "builtin");
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
        int ur = mpe_loader_unload(argv[2]);
        if (ur == 0) term_ok("mpe: mod: unloaded\n");
        else if (ur == -2) {
            term_err("mpe: mod: unload refused (busy: detach/reset world slots first)\n");
        } else term_err("mpe: mod: unload failed (unknown handle)\n");
        return;
    }
    if (term_str_eq(argv[1], "attach") && argc >= 3) {
        const mpe_module_desc_t *d = mpe_find_module(argv[2]);
        if (!d) { term_err("mpe: mod: unknown module\n"); return; }
        /* Only tick-capable modules attach: shapes/stages have no hooks
         * and would pin the .so while doing nothing. */
        if (!d->pre_step && !d->post_step && !d->attach && !d->detach) {
            term_err("mpe: mod: not a tick module (no hooks; nothing to attach)\n");
            return;
        }
        physics_world *pw = physics_world_get_primary();
        for (int i = 0; i < (pw ? pw->tick_module_count : 0); i++) {
            if (pw->tick_modules[i] && term_str_eq(pw->tick_modules[i]->name, argv[2])) {
                term_ok("mpe: mod: already attached\n");
                return;
            }
        }
        int r = physics_world_attach_module(pw, d);
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

/* modinfo command — show detailed info about a module */
void cmd_modinfo(int argc, char **argv) {
    if (argc < 2) {
        term_err("mpe: modinfo: usage: modinfo <module-name>\n");
        return;
    }
    const mpe_module_desc_t *d = mpe_find_module(argv[1]);
    if (!d) {
        term_err("mpe: modinfo: module not found\n");
        return;
    }
    char buf[512];
    snprintf(buf, sizeof(buf), "name:        %s\n", d->name);
    term_out(buf);
    snprintf(buf, sizeof(buf), "version:     %s\n", d->version ? d->version : "?");
    term_out(buf);
    snprintf(buf, sizeof(buf), "kind:        %s\n", d->kind ? d->kind : "generic");
    term_out(buf);
    snprintf(buf, sizeof(buf), "deterministic: %s\n", d->deterministic ? "yes" : "no");
    term_out(buf);
    snprintf(buf, sizeof(buf), "abi:         %u\n", d->abi);
    term_out(buf);
    /* module-specific fields */
    if (d->kind && term_str_eq(d->kind, "shape")) {
        term_out("type:        shape (pair handler)\n");
    } else if (d->kind && term_str_eq(d->kind, "broadphase")) {
        term_out("type:        broadphase backend\n");
    } else if (d->kind && term_str_eq(d->kind, "solver")) {
        term_out("type:        solver backend\n");
    } else if (d->kind && term_str_eq(d->kind, "generic")) {
        if (d->pre_step) term_out("hooks:       pre_step\n");
        if (d->post_step) term_out("hooks:       post_step\n");
    }
    /* check if loaded (compare loader module NAMES, not paths). */
    for (int i = 0; i < mpe_loader_count(); i++) {
        const char *nm = mpe_loader_name_at(i);
        if (nm && term_str_eq(nm, d->name)) {
            snprintf(buf, sizeof(buf), "loaded:      yes (%s)\n", mpe_loader_path_at(i));
            term_out(buf);
            return;
        }
    }
    snprintf(buf, sizeof(buf), "loaded:      no\n");
    term_out(buf);
}
