#!/usr/bin/env bash

set -euo pipefail

# ==============================================================================
# 18_build_a3.sh
#
# Implements migration directive #18:
#   Compile check — Build A3
#
# Run this from the folder CONTAINING src/, e.g.:
#   ./18_build_a3.sh
# ==============================================================================

if [[ ! -d "src" ]]; then
    echo "Error: This script must be run from the folder containing src/." >&2
    echo "Current directory: $(pwd)" >&2
    exit 1
fi

if [[ ! -f "src/makefile" && ! -f "src/Makefile" ]]; then
    echo "Error: No makefile found inside src/." >&2
    exit 1
fi

if ! command -v pkg-config >/dev/null 2>&1; then
    echo "Error: pkg-config not found." >&2
    echo "Install build dependencies first." >&2
    echo "" >&2
    echo "Ubuntu/Debian:" >&2
    echo "    sudo apt update" >&2
    echo "    sudo apt install build-essential pkg-config libgtk-3-dev libepoxy-dev" >&2
    exit 1
fi

if ! pkg-config --exists gtk+-3.0 epoxy; then
    echo "Error: Missing required libraries." >&2
    echo "" >&2
    echo "pkg-config could not find both of these:" >&2
    echo "    gtk+-3.0" >&2
    echo "    epoxy" >&2
    echo "" >&2
    echo "Ubuntu/Debian:" >&2
    echo "    sudo apt update" >&2
    echo "    sudo apt install build-essential pkg-config libgtk-3-dev libepoxy-dev" >&2
    exit 1
fi

LOG_FILE="build_a3.log"

echo "==> Cleaning A3 build"
if ! make -C src clean >"$LOG_FILE" 2>&1; then
    echo "Error: make clean failed." >&2
    echo "Full log: ${LOG_FILE}" >&2
    echo "" >&2
    cat "$LOG_FILE" >&2
    exit 1
fi

echo "==> Building A3"
echo "    Full build log: ${LOG_FILE}"

if ! make -C src >>"$LOG_FILE" 2>&1; then
    echo "" >&2
    echo "Build failed." >&2
    echo "Last 80 lines of ${LOG_FILE}:" >&2
    echo "" >&2
    tail -n 80 "$LOG_FILE" >&2
    exit 1
fi

if [[ ! -f "src/engine" ]]; then
    echo "Error: Build reported success, but src/engine was not found." >&2
    exit 1
fi

echo ""
echo "Build succeeded."
echo "Binary: src/engine"
echo "Log:    ${LOG_FILE}"
