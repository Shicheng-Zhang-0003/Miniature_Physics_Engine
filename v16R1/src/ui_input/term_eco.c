/* `eco` terminal command — drive loaded ecosystem bundles.
 * Streamlined 2026-10-04 (v15S release blocker): three verbs.
 *   eco attach <name>                   attach ecosystem to the primary world
 *   eco detach <name>                   detach ecosystem from the primary world
 *   eco command <name> <cmd> [args...]  run a bundle command (see `eco command <name> help`)
 * Bundles load via `mod load ecosystem/<member>/.../<name>.so`.
 *
 * PARKED (not deleted): `eco ls` (use `mod ls` for loaded handles) and
 * `eco config` (the only bundle key read parked game state). The C API
 * (mpe_ecosystem_* + config_get/set) is untouched.
 */
#include "../core/mpe_platform.h"
#include "term_priv.h"
#include "../core/mpe_loader.h"
#include "../ecosystem/mpe_ecosystem.h"
#include "../core/physics_world.h"
#include <stdio.h>
#include <string.h>
void cmd_eco (int argc, char **argv) {
    if (argc < 2) {
        term_err ("mpe: eco: usage: eco attach|detach|command\n");
        return;
    }
    if (term_str_eq (argv [1], "attach") && argc >= 3) {
        if (mpe_ecosystem_attach (physics_world_get_primary (), argv [2]) == 0) { term_ok ("mpe: eco: attached\n"); } else {
            term_err ("mpe: eco: attach failed (unknown ecosystem or no slot)\n");
        } return;
    }
    if (term_str_eq (argv [1], "detach") && argc >= 3) {
        if (mpe_ecosystem_detach (physics_world_get_primary (), argv [2]) == 0) { term_ok ("mpe: eco: detached\n"); } else {
            term_err ("mpe: eco: detach failed (not attached)\n");
        } return;
    }
    if (term_str_eq (argv [1], "command") && argc >= 4) {
        physics_world *w = physics_world_get_primary ();
        const mpe_ecosystem_desc_t *d = mpe_ecosystem_find (argv [2]);
        if (!d) {
            term_err ("mpe: eco: unknown ecosystem\n");
            return;
        }
        if (!d -> command) {
            term_err ("mpe: eco: bundle has no command interface\n");
            return;
        } void *st = mpe_ecosystem_state (w, argv [2]);
        if (!st) {
            term_err ("mpe: eco: not attached (eco attach first)\n");
            return;
        }
        if (d -> command (st, argc - 3, &argv [3]) == 0) { term_ok ("mpe: eco: ok\n"); } else {
            term_err ("mpe: eco: command failed (try: eco command <name> help)\n");
        } return;
    } term_err ("mpe: eco: unknown subcommand (attach|detach|command)\n");
}
