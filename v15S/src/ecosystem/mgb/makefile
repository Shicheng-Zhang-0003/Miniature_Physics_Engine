# mgb Makefile — 472-MGB core + MPE sink + s2-bridge module + headless tests.
#
# Standalone from the engine build (mirrors mfs dual-mode): SRC auto-detects
# v15S/src from this directory; override with `make SRC=/path`.
# Targets: all / test (headless tiers A+B) / module (.so) / loadtest / clean.
# Graphical testing is explicitly OUT (owner runs it personally, later).
CC      = gcc
MGB     := $(shell cd "$(dirname $(lastword $(MAKEFILE_LIST)))" && pwd)
SRC     ?= $(MGB)/../..
CFLAGS  = -O2 -g -Wall -Wextra -std=c11 -I$(MGB) -I$(MGB)/include \
          -I$(SRC) -ffp-contract=off
PIC     = -fPIC
OUT     = $(MGB)/build
DL      = -ldl
EPOXY   = $(shell pkg-config --libs epoxy 2>/dev/null)

# Engine core objects (same proven list as mfs build_tests.sh CORE).
CORE = $(SRC)/core/physics_world.c $(SRC)/core/rigidbody.c \
       $(SRC)/core/mpe_registry.c $(SRC)/core/mpe_loader.c \
       $(SRC)/core/mpe_diag.c $(SRC)/core/event_log.c \
       $(SRC)/core/det_math.c $(SRC)/core/mpe_primary.c \
       $(SRC)/physics/collision_narrowphase.c $(SRC)/physics/collision_cache.c \
       $(SRC)/physics/collision_solver.c $(SRC)/physics/collision_ccd.c \
       $(SRC)/physics/collision_cylinder.c $(SRC)/physics/broadphase.c \
       $(SRC)/physics/constraint.c $(SRC)/physics/revolute_joint.c \
       $(SRC)/physics/depenetration.c $(SRC)/physics/islands.c \
       $(SRC)/config/mpe_config.c $(SRC)/config/mpe_config_schema.c \
       $(SRC)/scene/boundary.c $(SRC)/ecosystem/mpe_ecosystem.c \
       $(SRC)/physics/spring_joint.c

MGB_CORE = $(MGB)/src/mgb_link.c $(MGB)/src/mgb_frame.c \
           $(MGB)/src/mgb_map.c $(MGB)/src/mgb_sink_file.c \
           $(MGB)/src/mgb_sink_mpe.c
MGB_MOD = $(MGB)/modules/s2/s2_bridge.c
MGB_MOD_OBJ = $(OUT)/s2_bridge.o

.PHONY: all test module loadtest clean

all: $(OUT) $(OUT)/mgb_mpe_test module

$(OUT):
	mkdir -p $(OUT)

$(OUT)/s2_bridge.o: $(MGB_MOD)
	$(CC) $(CFLAGS) -c $< -o $@

$(OUT)/mgb_mpe_test: $(MGB)/tests/mgb_mpe_test.c $(MGB_CORE) $(MGB_MOD_OBJ) $(CORE)
	$(CC) $(CFLAGS) $(MGB)/tests/mgb_mpe_test.c $(MGB_CORE) $(MGB_MOD_OBJ) $(CORE) \
	    -lm $(DL) $(EPOXY) -o $@

# Thin plugin (MFS rule): module objects only; engine symbols resolve
# against the host at dlopen (host links -rdynamic).
module: $(OUT)/mgb_bridge.so

$(OUT)/mgb_bridge.so: $(MGB_CORE) $(MGB_MOD)
	$(CC) $(CFLAGS) $(PIC) -shared $(MGB_CORE) $(MGB_MOD) -lm -o $@

# Headless .so check: descriptor dynamically exported (engine symbols
# resolve against the host at mod-load time, so no dlopen here).
loadtest: all
	nm -D --defined-only $(OUT)/mgb_bridge.so | grep -q " mpe_module_desc$$" && \
	    echo "  PASS mpe_module_desc exported" || \
	    (echo "  FAIL mpe_module_desc missing"; exit 1)

# Headless tiers: A always runs; B needs s2tui (MGB_S2TUI_BIN or sibling).
# Usage: make test [MGB_S2TUI_BIN=/path/to/s2tui] [MGBDIR=/abs/mgb]
test: all
	cd $(MGB) && $(OUT)/mgb_mpe_test $(MGB)

clean:
	rm -rf $(OUT)
