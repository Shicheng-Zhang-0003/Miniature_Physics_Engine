# A3_PATCH_31_INCREMENTAL_MAKEFILE
CC = gcc
# A3_PATCH_32_PORTABLE_GRID_RESTORE
ENABLE_NATIVE ?= 0
NATIVE_FLAG := $(if $(filter 1,$(ENABLE_NATIVE)),-march=native,)
CFLAGS = $(shell pkg-config --cflags gtk+-3.0 epoxy) -I. -O3 $(NATIVE_FLAG) -Wall -Wextra -MMD -MP
LIBS = $(shell pkg-config --libs gtk+-3.0 epoxy) -lm

BUILD_DIR := build
SOURCES := $(shell find . -name '*.c' | sed 's|^\./||')
OBJECTS := $(patsubst %.c,$(BUILD_DIR)/%.o,$(SOURCES))
DEPS := $(OBJECTS:.o=.d)

engine: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) $(LIBS) -o engine

$(BUILD_DIR)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR) engine

.PHONY: clean

-include $(DEPS)
