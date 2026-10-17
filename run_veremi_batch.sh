#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
EXP_DIR="$SCRIPT_DIR/detection-eval"
BUILD_DIR="$EXP_DIR/build"
RUNNER="$BUILD_DIR/veremi_runner"

# Run the batch sweep specifically for the VeReMi extension experiment
cd "$EXP_DIR" || exit 1

# -----------------------------------------------------------------------------
# Python environment
# -----------------------------------------------------------------------------
if [ ! -d ".venv" ]; then
    echo "Creating virtual environment in $EXP_DIR..."
    python3 -m venv .venv
    if ! .venv/bin/pip install -r requirements.txt; then
        echo "Error: failed to install Python dependencies. Removing incomplete .venv." >&2
        rm -rf .venv
        exit 1
    fi
fi

if [ "${VEREMI_SKIP_BUILD:-0}" = "1" ]; then
    if [ ! -x "$RUNNER" ]; then
        echo "Error: VEREMI_SKIP_BUILD=1 but $RUNNER does not exist." >&2
        exit 1
    fi
else
    for tool in cmake make c++; do
        if ! command -v "$tool" &> /dev/null; then
            echo "Error: '$tool' not found in PATH. Install a C++ toolchain and CMake" >&2
            echo "       (and Apache Arrow/Parquet: libarrow-dev, libparquet-dev)." >&2
            exit 1
        fi
    done

    if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
        echo "Configuring veremi_runner build in $BUILD_DIR..."
        if ! cmake -S "$EXP_DIR" -B "$BUILD_DIR" \
                   -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-}"; then
            echo "Error: CMake configuration failed." >&2
            rm -rf "$BUILD_DIR"
            exit 1
        fi
    fi

    echo "Building veremi_runner..."
    if ! cmake --build "$BUILD_DIR" -j "$(nproc)"; then
        echo "Error: failed to build veremi_runner." >&2
        echo "       Apache Arrow and Parquet development libraries are required" >&2
        echo "       (libarrow-dev, libparquet-dev; 'pkg-config --modversion arrow parquet')." >&2
        exit 1
    fi

    if [ ! -x "$RUNNER" ]; then
        echo "Error: build reported success but $RUNNER is missing." >&2
        exit 1
    fi
fi

.venv/bin/python3 scripts/run_veremi_batch.py "$@"
