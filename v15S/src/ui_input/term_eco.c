/* `eco` terminal command — drive loaded ecosystem bundles.
 * Usage:
 *   eco ls                              list registered ecosystems + primary attachments
 *   eco attach <name>                   attach ecosystem to the primary world
 *   eco detach <name>                   detach ecosystem from the primary world
 *   eco command <name> <cmd> [args...]  run a bundle command (see `eco command <name> help`)
 *   eco config <name> get <key>         read a bundle config key
 *   eco config <name> set <key> <value> write a bundle config key
 * Bundles load via `mod load ecosystem/mfs/<name>.so` (or plugins/<name>.so).
 */
#include "term_priv.h"
#include "../core/mpe_loader.h"
#include "../ecosystem/mpe_ecosystem.h"
#include "../core/physics_world.h"
#include <stdio.h>
#include <string.h>

void cmd_eco(int argc, char **argv) {
    if (argc < 2) {
        term_err("mpe: eco: usage: eco ls|attach|detach|command|config\n");
        return;
    }
    if (term_str_eq(argv[1], "ls")) {
        char buf[256];
        term_out("ecosystems:\n");
        for (int i = 0; i < mpe_ecosystem_count(); i++) {
            const mpe_ecosystem_desc_t *d = mpe_ecosystem_at(i);
            if (!d) continue;
            snprintf(buf, sizeof(buf), "  %s-%s%s\n", d->name, d->version ? d->version : "?",
                     d->deterministic ? " det" : "");
            term_out(buf);
        }
        physics_world *w = physics_world_get_primary();
        snprintf(buf, sizeof(buf), "loaded .so handles: %d\n", mpe_loader_count());
        term_out(buf);
        for (int i = 0; i < mpe_loader_count(); i++) {
            const char *nm = mpe_loader_name_at(i);
            snprintf(buf, sizeof(buf), "  %s (%s)\n", mpe_loader_path_at(i), nm ? nm : "?");
            term_out(buf);
        }
        if (w) {
            term_out("attached to primary:\n");
            for (int i = 0; i < mpe_ecosystem_count(); i++) {
                const mpe_ecosystem_desc_t *d = mpe_ecosystem_at(i);
                if (d && mpe_ecosystem_state(w, d->name)) {
                    snprintf(buf, sizeof(buf), "  %s\n", d->name);
                    term_out(buf);
                }
            }
        }
        return;
    }
    if (term_str_eq(argv[1], "attach") && argc >= 3) {
        if (mpe_ecosystem_attach(physics_world_get_primary(), argv[2]) == 0) {
            term_ok("mpe: eco: attached\n");
        } else {
            term_err("mpe: eco: attach failed (unknown ecosystem or no slot)\n");
        }
        return;
    }
    if (term_str_eq(argv[1], "detach") && argc >= 3) {
        if (mpe_ecosystem_detach(physics_world_get_primary(), argv[2]) == 0) {
            term_ok("mpe: eco: detached\n");
        } else {
            term_err("mpe: eco: detach failed (not attached)\n");
        }
        return;
    }
    if (term_str_eq(argv[1], "command") && argc >= 4) {
        physics_world *w = physics_world_get_primary();
        const mpe_ecosystem_desc_t *d = mpe_ecosystem_find(argv[2]);
        if (!d) {
            term_err("mpe: eco: unknown ecosystem\n");
            return;
        }
        if (!d->command) {
            term_err("mpe: eco: bundle has no command interface\n");
            return;
        }
        void *st = mpe_ecosystem_state(w, argv[2]);
        if (!st) {
            term_err("mpe: eco: not attached (eco attach first)\n");
            return;
        }
        if (d->command(st, argc - 3, &argv[3]) == 0) {
            term_ok("mpe: eco: ok\n");
        } else {
            term_err("mpe: eco: command failed (try: eco command <name> help)\n");
        }
        return;
    }
    if (term_str_eq(argv[1], "config") && argc >= 5) {
        physics_world *w = physics_world_get_primary();
        const mpe_ecosystem_desc_t *d = mpe_ecosystem_find(argv[2]);
        if (!d) {
            term_err("mpe: eco: unknown ecosystem\n");
            return;
        }
        void *st = mpe_ecosystem_state(w, argv[2]);
        if (!st) {
            term_err("mpe: eco: not attached (eco attach first)\n");
            return;
        }
        if (term_str_eq(argv[3], "get") && argc >= 5) {
            char out[256] = {0};
            if (!d->config_get) {
                term_err("mpe: eco: bundle has no config interface\n");
                return;
            }
            if (d->config_get(st, argv[4], out, sizeof(out)) == 0) {
                char buf[300];
                snprintf(buf, sizeof(buf), "%s\n", out);
                term_out(buf);
            } else {
                term_err("mpe: eco: unknown key\n");
            }
            return;
        }
        if (term_str_eq(argv[3], "set") && argc >= 6) {
            if (!d->config_set) {
                term_err("mpe: eco: bundle has no config interface\n");
                return;
            }
            if (d->config_set(st, argv[4], argv[5]) == 0) {
                term_ok("mpe: eco: ok\n");
            } else {
                term_err("mpe: eco: unsupported key\n");
            }
            return;
        }
        term_err("mpe: eco: usage: eco config <name> get <key> | set <key> <value>\n");
        return;
    }
    term_err("mpe: eco: unknown subcommand\n");
}
